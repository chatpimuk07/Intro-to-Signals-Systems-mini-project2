-- =========================================================================
-- Signals & Systems / DSP Practical Labs
-- Task 3 (v2) : Audio Sampling using MAX 10 FPGA ADC + Analog Microphone
-- File        : task3_top.vhd   (top-level entity, Terasic DE10-Lite)
--
--   MAX10_CLK1_50 --> sample_tick_gen (16 kHz) --+--> mcp3201_ctrl  --+
--                                                |   (or MAX10 ADC)   |
--                                                +--> test_tone_dds --+--> mux (SW0)
--                                                                         |
--                         ESP32 (SPI master) <-- spi_slave_tx <-- pingpong_buffer
--                                         <-- FPGA_DATA_READY ------------+
--
--   SW[0]    : 0 = ADC (microphone / function generator), 1 = DDS test tone
--   SW[3:1]  : test-tone frequency (see sine_lut_pkg)
--   KEY[0]   : reset (active low)
--   LEDR[0]  : unread block available (FPGA_DATA_READY)
--   LEDR[1]  : test-tone mode
--   LEDR[9:2]: peak-level bar graph (per block)
--   TIMING_DEBUG_PIN : toggles at every sample tick -> 8.000 kHz square wave
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity task3_top is
  generic (
    CLK_HZ            : positive := 50_000_000;
    FS_HZ             : positive := 16_000;
    ADC_SCLK_HZ       : positive := 1_000_000;   -- MCP3201 clock (<=1 MHz @ 3.3 V)
    ADDR_BITS         : positive := 10;          -- N = 1024 samples / block
    USE_MAX10_ADC     : boolean  := false;       -- true = on-chip ADC (needs IP)
    MAX10_ADC_CHANNEL : natural  := 1            -- 1 = Arduino A0
  );
  port (
    MAX10_CLK1_50    : in  std_logic;
    KEY              : in  std_logic_vector(1 downto 0);
    SW               : in  std_logic_vector(9 downto 0);
    LEDR             : out std_logic_vector(9 downto 0);
    -- external MCP3201 ADC (Arduino header IO2..IO4)
    ADC_SPI_CS_N     : out std_logic;
    ADC_SPI_SCK      : out std_logic;
    ADC_SPI_MISO     : in  std_logic;
    -- ESP32 link (Arduino header IO9, IO10, IO12, IO13)
    ESP_SPI_SCK      : in  std_logic;
    ESP_SPI_CS_N     : in  std_logic;
    ESP_SPI_MISO     : out std_logic;
    FPGA_DATA_READY  : out std_logic;
    -- oscilloscope probe (Arduino header IO8)
    TIMING_DEBUG_PIN : out std_logic
  );
end entity task3_top;

architecture rtl of task3_top is
  signal clk        : std_logic;
  signal rst_sr     : std_logic_vector(2 downto 0) := (others => '1');
  signal rst        : std_logic;
  signal sw_sync1   : std_logic_vector(3 downto 0) := (others => '0');
  signal sw_sync2   : std_logic_vector(3 downto 0) := (others => '0');
  signal test_mode  : std_logic;
  signal tone_sel   : std_logic_vector(2 downto 0);

  signal tick       : std_logic;
  signal tick_tog   : std_logic;

  signal adc_data   : std_logic_vector(11 downto 0);
  signal adc_valid  : std_logic;
  signal tone_data  : std_logic_vector(11 downto 0);
  signal tone_valid : std_logic;

  signal smp_data   : std_logic_vector(11 downto 0);
  signal smp_valid  : std_logic;
  signal wr_word    : std_logic_vector(15 downto 0);

  signal rd_bank    : std_logic;
  signal rd_addr    : std_logic_vector(ADDR_BITS - 1 downto 0);
  signal rd_data    : std_logic_vector(15 downto 0);
  signal ready_bank : std_logic;
  signal data_ready : std_logic;
  signal block_done : std_logic;
  signal block_seq  : std_logic_vector(15 downto 0);
  signal overruns   : std_logic_vector(7 downto 0);
  signal frame_ack  : std_logic;
  signal status     : std_logic_vector(15 downto 0);

  signal level      : unsigned(10 downto 0);
  signal peak       : unsigned(10 downto 0) := (others => '0');
  signal peak_disp  : unsigned(10 downto 0) := (others => '0');
  signal bar        : std_logic_vector(7 downto 0);
begin
  clk <= MAX10_CLK1_50;

  ---------------------------------------------------------------------------
  -- reset (KEY0, active low) and switch synchronisers
  ---------------------------------------------------------------------------
  process (clk)
  begin
    if rising_edge(clk) then
      rst_sr   <= rst_sr(1 downto 0) & (not KEY(0));
      sw_sync1 <= SW(3 downto 0);
      sw_sync2 <= sw_sync1;
    end if;
  end process;
  rst       <= rst_sr(2);
  test_mode <= sw_sync2(0);
  tone_sel  <= sw_sync2(3 downto 1);

  ---------------------------------------------------------------------------
  -- 1) hardware sampling clock-enable : Fs = 16 kS/s
  ---------------------------------------------------------------------------
  u_tick : entity work.sample_tick_gen
    generic map (CLK_HZ => CLK_HZ, FS_HZ => FS_HZ)
    port map (clk => clk, rst => rst, tick => tick, tick_toggle => tick_tog);

  TIMING_DEBUG_PIN <= tick_tog;

  ---------------------------------------------------------------------------
  -- 2) ADC : external MCP3201 (default) or MAX 10 on-chip ADC
  ---------------------------------------------------------------------------
  g_mcp : if not USE_MAX10_ADC generate
    u_adc : entity work.mcp3201_ctrl
      generic map (CLK_HZ => CLK_HZ, SCLK_HZ => ADC_SCLK_HZ)
      port map (
        clk      => clk,
        rst      => rst,
        start    => tick,
        adc_cs_n => ADC_SPI_CS_N,
        adc_sclk => ADC_SPI_SCK,
        adc_miso => ADC_SPI_MISO,
        data     => adc_data,
        valid    => adc_valid,
        busy     => open
      );
  end generate;

  g_max10 : if USE_MAX10_ADC generate
    u_adc : entity work.max10_adc_sampler
      generic map (ADC_CHANNEL => MAX10_ADC_CHANNEL)
      port map (clk => clk, rst => rst, start => tick,
                data => adc_data, valid => adc_valid);
    ADC_SPI_CS_N <= '1';
    ADC_SPI_SCK  <= '0';
  end generate;

  ---------------------------------------------------------------------------
  -- 3) built-in DDS test tone
  ---------------------------------------------------------------------------
  u_tone : entity work.test_tone_dds
    port map (clk => clk, rst => rst, tick => tick, freq_sel => tone_sel,
              data => tone_data, valid => tone_valid);

  smp_data  <= tone_data  when test_mode = '1' else adc_data;
  smp_valid <= tone_valid when test_mode = '1' else adc_valid;
  wr_word   <= "0000" & smp_data;

  ---------------------------------------------------------------------------
  -- 4) Block-RAM ping-pong buffer (2 x 1024 samples)
  ---------------------------------------------------------------------------
  u_buf : entity work.pingpong_buffer
    generic map (ADDR_BITS => ADDR_BITS, DATA_BITS => 16)
    port map (
      clk        => clk,
      rst        => rst,
      wr_en      => smp_valid,
      wr_data    => wr_word,
      rd_bank    => rd_bank,
      rd_addr    => rd_addr,
      rd_data    => rd_data,
      ack        => frame_ack,
      ready_bank => ready_bank,
      data_ready => data_ready,
      block_done => block_done,
      block_seq  => block_seq,
      overruns   => overruns
    );

  ---------------------------------------------------------------------------
  -- 5) SPI slave to ESP32
  ---------------------------------------------------------------------------
  status(15)           <= test_mode;
  status(14)           <= '1' when USE_MAX10_ADC else '0';
  status(13 downto 11) <= tone_sel;
  status(10 downto 8)  <= (others => '0');
  status(7 downto 0)   <= overruns;

  u_spi : entity work.spi_slave_tx
    generic map (ADDR_BITS => ADDR_BITS)
    port map (
      clk        => clk,
      rst        => rst,
      spi_sck    => ESP_SPI_SCK,
      spi_cs_n   => ESP_SPI_CS_N,
      spi_miso   => ESP_SPI_MISO,
      ready_bank => ready_bank,
      block_seq  => block_seq,
      status     => status,
      rd_bank    => rd_bank,
      rd_addr    => rd_addr,
      rd_data    => rd_data,
      frame_ack  => frame_ack,
      active     => open
    );

  FPGA_DATA_READY <= data_ready;

  ---------------------------------------------------------------------------
  -- 6) LEDs : status + per-block peak level bar graph
  ---------------------------------------------------------------------------
  level <= unsigned(smp_data(10 downto 0)) when smp_data(11) = '1'
           else to_unsigned(2047, 11) - unsigned(smp_data(10 downto 0));

  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        peak      <= (others => '0');
        peak_disp <= (others => '0');
      elsif block_done = '1' then
        peak_disp <= peak;
        peak      <= (others => '0');
      elsif smp_valid = '1' and level > peak then
        peak <= level;
      end if;
    end if;
  end process;

  bar(0) <= '1' when peak_disp > 8    else '0';
  bar(1) <= '1' when peak_disp > 16   else '0';
  bar(2) <= '1' when peak_disp > 32   else '0';
  bar(3) <= '1' when peak_disp > 64   else '0';
  bar(4) <= '1' when peak_disp > 128  else '0';
  bar(5) <= '1' when peak_disp > 256  else '0';
  bar(6) <= '1' when peak_disp > 512  else '0';
  bar(7) <= '1' when peak_disp > 1536 else '0';    -- close to clipping

  LEDR(0)          <= data_ready;
  LEDR(1)          <= test_mode;
  LEDR(9 downto 2) <= bar;
end architecture rtl;
