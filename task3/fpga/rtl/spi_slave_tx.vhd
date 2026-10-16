-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : spi_slave_tx.vhd
-- Purpose : SPI slave (ESP32 = master, FPGA = slave), SPI mode 0
--           (CPOL=0, CPHA=0, MSB first).  Streams one complete block frame
--           per chip-select transaction:
--
--   byte : 0     1     2..3        4..5         6..7        8 .. 8+2N-1
--          0xA5  0x5A  block_seq   N (=1024)    status      N x uint16
--          (all multi-byte fields big-endian; samples = 0000 b11..b0)
--
--   status[15]   = 1 : built-in DDS test tone, 0 : ADC
--   status[14]   = 1 : MAX 10 on-chip ADC,    0 : external MCP3201
--   status[13:11]= SW[3:1] tone select
--   status[7:0]  = block overrun counter (saturating)
--
-- SCK / CS_N are asynchronous to the 50 MHz clock and are oversampled
-- with 3-FF synchronisers (recommended SCK <= 4 MHz).  The bank, sequence
-- number and status are latched at the falling edge of CS_N so a frame is
-- always self-consistent even if a new block completes during the read.
-- After CS_N rises with >= FRAME_BYTES bytes clocked, 'frame_ack' pulses.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_slave_tx is
  generic (
    ADDR_BITS : positive := 10
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;
    -- SPI pins
    spi_sck    : in  std_logic;
    spi_cs_n   : in  std_logic;
    spi_miso   : out std_logic;
    -- block information from the ping-pong buffer
    ready_bank : in  std_logic;
    block_seq  : in  std_logic_vector(15 downto 0);
    status     : in  std_logic_vector(15 downto 0);
    -- buffer read port
    rd_bank    : out std_logic;
    rd_addr    : out std_logic_vector(ADDR_BITS - 1 downto 0);
    rd_data    : in  std_logic_vector(15 downto 0);
    -- handshake
    frame_ack  : out std_logic;
    active     : out std_logic
  );
end entity spi_slave_tx;

architecture rtl of spi_slave_tx is
  constant N_SAMPLES   : positive := 2 ** ADDR_BITS;
  constant HDR_BYTES   : positive := 8;
  constant FRAME_BYTES : positive := HDR_BYTES + 2 * N_SAMPLES;    -- 2056
  constant IDX_MAX     : positive := 2 * FRAME_BYTES;               -- saturation

  signal sck_sr   : std_logic_vector(2 downto 0) := (others => '0');
  signal cs_sr    : std_logic_vector(2 downto 0) := (others => '1');
  signal sck_rise : std_logic;
  signal sck_fall : std_logic;
  signal cs_fall  : std_logic;
  signal cs_rise  : std_logic;
  signal cs_act   : std_logic;

  signal byte_idx : natural range 0 to IDX_MAX := 0;
  signal bit_cnt  : natural range 0 to 7 := 0;
  signal sh       : std_logic_vector(7 downto 0) := (others => '0');
  signal cur_byte : std_logic_vector(7 downto 0);

  signal bank_l   : std_logic := '0';
  signal seq_l    : std_logic_vector(15 downto 0) := (others => '0');
  signal stat_l   : std_logic_vector(15 downto 0) := (others => '0');
  signal ack_r    : std_logic := '0';
  signal samp_idx : unsigned(ADDR_BITS - 1 downto 0);
begin
  ---------------------------------------------------------------------------
  -- synchronisers & edge detectors
  ---------------------------------------------------------------------------
  sync_proc : process (clk)
  begin
    if rising_edge(clk) then
      sck_sr <= sck_sr(1 downto 0) & spi_sck;
      cs_sr  <= cs_sr(1 downto 0) & spi_cs_n;
    end if;
  end process;

  sck_rise <= '1' when sck_sr(2 downto 1) = "01" else '0';
  sck_fall <= '1' when sck_sr(2 downto 1) = "10" else '0';
  cs_fall  <= '1' when cs_sr(2 downto 1) = "10" else '0';
  cs_rise  <= '1' when cs_sr(2 downto 1) = "01" else '0';
  cs_act   <= not cs_sr(1);

  ---------------------------------------------------------------------------
  -- byte multiplexer : header bytes or sample bytes from block RAM.
  -- The RAM address follows byte_idx, so the next sample word is already
  -- available (1-clock latency) long before the SCK edge that needs it.
  ---------------------------------------------------------------------------
  samp_idx <= to_unsigned(((byte_idx - HDR_BYTES) / 2) mod N_SAMPLES, ADDR_BITS)
              when byte_idx >= HDR_BYTES else (others => '0');
  rd_addr  <= std_logic_vector(samp_idx);
  rd_bank  <= bank_l;

  mux_proc : process (byte_idx, seq_l, stat_l, rd_data)
  begin
    case byte_idx is
      when 0      => cur_byte <= x"A5";
      when 1      => cur_byte <= x"5A";
      when 2      => cur_byte <= seq_l(15 downto 8);
      when 3      => cur_byte <= seq_l(7 downto 0);
      when 4      => cur_byte <= std_logic_vector(to_unsigned(N_SAMPLES / 256, 8));
      when 5      => cur_byte <= std_logic_vector(to_unsigned(N_SAMPLES mod 256, 8));
      when 6      => cur_byte <= stat_l(15 downto 8);
      when 7      => cur_byte <= stat_l(7 downto 0);
      when others =>
        if (byte_idx mod 2) = 0 then
          cur_byte <= rd_data(15 downto 8);         -- high byte first
        else
          cur_byte <= rd_data(7 downto 0);
        end if;
    end case;
  end process;

  ---------------------------------------------------------------------------
  -- SPI mode 0 shifter
  --   master samples MISO on SCK rising edge, slave updates on falling edge
  ---------------------------------------------------------------------------
  spi_proc : process (clk)
  begin
    if rising_edge(clk) then
      ack_r <= '0';
      if rst = '1' then
        byte_idx <= 0;
        bit_cnt  <= 0;
        sh       <= (others => '0');
      elsif cs_fall = '1' then
        -- start of transaction: snapshot block information
        bank_l   <= ready_bank;
        seq_l    <= block_seq;
        stat_l   <= status;
        byte_idx <= 0;
        bit_cnt  <= 0;
        sh       <= x"A5";                          -- first bit valid before 1st SCK
      elsif cs_act = '1' then
        if sck_rise = '1' then
          if bit_cnt = 7 then
            bit_cnt <= 0;
            if byte_idx < IDX_MAX then
              byte_idx <= byte_idx + 1;
            end if;
          else
            bit_cnt <= bit_cnt + 1;
          end if;
        elsif sck_fall = '1' then
          if bit_cnt = 0 then
            sh <= cur_byte;                         -- next byte, MSB first
          else
            sh <= sh(6 downto 0) & '0';
          end if;
        end if;
      end if;

      if cs_rise = '1' and byte_idx >= FRAME_BYTES then
        ack_r <= '1';
      end if;
    end if;
  end process;

  spi_miso  <= sh(7) when cs_act = '1' else 'Z';
  frame_ack <= ack_r;
  active    <= cs_act;
end architecture rtl;
