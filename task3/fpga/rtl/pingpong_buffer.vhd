-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : pingpong_buffer.vhd
-- Purpose : Block-RAM ping-pong (double) sample buffer.
--
--   One inferred simple-dual-port RAM of 2 x 2^ADDR_BITS words (2 x 1024 x 16
--   = 32 Kbit -> 4 M9K blocks).  The MSB of the address selects the bank.
--     * write side : the sampler writes bank 'wr_bank' at index 0..N-1;
--                    when index N-1 is written, the bank is complete,
--                    'ready_bank' <= wr_bank, block_seq++, data_ready <= '1'
--                    and writing continues seamlessly in the other bank.
--     * read side  : the SPI slave reads any bank/address (1-cycle latency).
--                    After a complete frame was read it pulses 'ack', which
--                    clears data_ready.
--   If a new block completes while data_ready is still '1' (ESP32 too slow)
--   the overrun counter is incremented (saturating) - sampling never stops.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pingpong_buffer is
  generic (
    ADDR_BITS : positive := 10;          -- N = 2**ADDR_BITS samples per block
    DATA_BITS : positive := 16
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;
    -- write port (sampler)
    wr_en      : in  std_logic;
    wr_data    : in  std_logic_vector(DATA_BITS - 1 downto 0);
    -- read port (SPI slave)
    rd_bank    : in  std_logic;
    rd_addr    : in  std_logic_vector(ADDR_BITS - 1 downto 0);
    rd_data    : out std_logic_vector(DATA_BITS - 1 downto 0);
    ack        : in  std_logic;          -- frame fully read
    -- status
    ready_bank : out std_logic;          -- most recently completed bank
    data_ready : out std_logic;          -- an unread block is available
    block_done : out std_logic;          -- 1-clock pulse per completed block
    block_seq  : out std_logic_vector(15 downto 0);
    overruns   : out std_logic_vector(7 downto 0)
  );
end entity pingpong_buffer;

architecture rtl of pingpong_buffer is
  constant DEPTH : positive := 2 ** (ADDR_BITS + 1);
  type ram_t is array (0 to DEPTH - 1) of std_logic_vector(DATA_BITS - 1 downto 0);
  signal ram : ram_t;

  signal wr_bank   : std_logic := '0';
  signal wr_idx    : unsigned(ADDR_BITS - 1 downto 0) := (others => '0');
  signal wr_addr   : unsigned(ADDR_BITS downto 0);
  signal rd_addr_f : unsigned(ADDR_BITS downto 0);
  signal rd_q      : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');

  signal rdy_bank_r : std_logic := '1';
  signal rdy_r      : std_logic := '0';
  signal done_r     : std_logic := '0';
  signal seq_r      : unsigned(15 downto 0) := (others => '1');  -- first block = 0
  signal ovr_r      : unsigned(7 downto 0) := (others => '0');
begin
  wr_addr   <= wr_bank & wr_idx;
  rd_addr_f <= rd_bank & unsigned(rd_addr);

  -- RAM (inferred M9K, simple dual port, single clock)
  ram_proc : process (clk)
  begin
    if rising_edge(clk) then
      if wr_en = '1' then
        ram(to_integer(wr_addr)) <= wr_data;
      end if;
      rd_q <= ram(to_integer(rd_addr_f));
    end if;
  end process;

  -- bank / handshake control
  ctrl_proc : process (clk)
  begin
    if rising_edge(clk) then
      done_r <= '0';
      if rst = '1' then
        wr_bank    <= '0';
        wr_idx     <= (others => '0');
        rdy_bank_r <= '1';
        rdy_r      <= '0';
        seq_r      <= (others => '1');
        ovr_r      <= (others => '0');
      else
        if ack = '1' then
          rdy_r <= '0';
        end if;

        if wr_en = '1' then
          if wr_idx = to_unsigned(2 ** ADDR_BITS - 1, ADDR_BITS) then  -- last sample
            wr_idx     <= (others => '0');
            wr_bank    <= not wr_bank;
            rdy_bank_r <= wr_bank;
            seq_r      <= seq_r + 1;
            done_r     <= '1';
            rdy_r      <= '1';                       -- has priority over ack
            if rdy_r = '1' and ack = '0' and ovr_r /= x"FF" then
              ovr_r <= ovr_r + 1;
            end if;
          else
            wr_idx <= wr_idx + 1;
          end if;
        end if;
      end if;
    end if;
  end process;

  rd_data    <= rd_q;
  ready_bank <= rdy_bank_r;
  data_ready <= rdy_r;
  block_done <= done_r;
  block_seq  <= std_logic_vector(seq_r);
  overruns   <= std_logic_vector(ovr_r);
end architecture rtl;
