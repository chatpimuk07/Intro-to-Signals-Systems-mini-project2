-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : max10_adc_sampler.vhd
-- Purpose : OPTIONAL alternative to the external MCP3201 - uses the MAX 10
--           on-chip ADC (Modular ADC core, "ADC control core only") and
--           triggers exactly one conversion per sample tick through the
--           Avalon-ST command interface.
--
-- Only used when task3_top generic USE_MAX10_ADC = true.  Before that you
-- must generate two IP variations with the Quartus IP Catalog (see README):
--   1. ALTPLL           named  adc_pll    : inclk0 = 50 MHz, c0 = 10 MHz, locked
--   2. Modular ADC core named  max10_adc  : "ADC control core only",
--                                            ADC clock from PLL (10 MHz),
--                                            channel CH1 (= Arduino A0) enabled
-- and add the generated .qip files to the project.
--
-- DE10-Lite: Arduino A0..A5 = ADC1 CH1..CH6, header range 0..5 V (scaled to
-- the 0..2.5 V internal ADC range by an on-board divider/op-amp).
--
-- Because the 50 MHz system clock and the 10 MHz ADC clock come from the
-- same PLL and 3125 x 20 ns = 625 x 100 ns exactly, every conversion starts
-- at the same phase relative to the tick -> no sampling jitter.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity max10_adc_sampler is
  generic (
    ADC_CHANNEL : natural range 0 to 16 := 1        -- 1 = Arduino A0
  );
  port (
    clk   : in  std_logic;                          -- 50 MHz
    rst   : in  std_logic;
    start : in  std_logic;                          -- sample tick
    data  : out std_logic_vector(11 downto 0);
    valid : out std_logic
  );
end entity max10_adc_sampler;

architecture rtl of max10_adc_sampler is
  component adc_pll is
    port (
      inclk0 : in  std_logic;
      c0     : out std_logic;
      locked : out std_logic
    );
  end component;

  component max10_adc is
    port (
      clock_clk              : in  std_logic;
      reset_sink_reset_n     : in  std_logic;
      adc_pll_clock_clk      : in  std_logic;
      adc_pll_locked_export  : in  std_logic;
      command_valid          : in  std_logic;
      command_channel        : in  std_logic_vector(4 downto 0);
      command_startofpacket  : in  std_logic;
      command_endofpacket    : in  std_logic;
      command_ready          : out std_logic;
      response_valid         : out std_logic;
      response_channel       : out std_logic_vector(4 downto 0);
      response_data          : out std_logic_vector(11 downto 0);
      response_startofpacket : out std_logic;
      response_endofpacket   : out std_logic
    );
  end component;

  constant CMD_CHANNEL : std_logic_vector(4 downto 0) :=
    std_logic_vector(to_unsigned(ADC_CHANNEL, 5));

  signal clk_10m    : std_logic;
  signal pll_locked : std_logic;
  signal rst_n      : std_logic;
  signal cmd_valid  : std_logic := '0';
  signal cmd_ready  : std_logic;
  signal rsp_valid  : std_logic;
  signal rsp_chan   : std_logic_vector(4 downto 0);
  signal rsp_data   : std_logic_vector(11 downto 0);
  signal data_r     : std_logic_vector(11 downto 0) := (others => '0');
  signal valid_r    : std_logic := '0';
begin
  rst_n <= not rst;

  u_pll : adc_pll
    port map (inclk0 => clk, c0 => clk_10m, locked => pll_locked);

  u_adc : max10_adc
    port map (
      clock_clk              => clk,
      reset_sink_reset_n     => rst_n,
      adc_pll_clock_clk      => clk_10m,
      adc_pll_locked_export  => pll_locked,
      command_valid          => cmd_valid,
      command_channel        => CMD_CHANNEL,
      command_startofpacket  => '1',
      command_endofpacket    => '1',
      command_ready          => cmd_ready,
      response_valid         => rsp_valid,
      response_channel       => rsp_chan,
      response_data          => rsp_data,
      response_startofpacket => open,
      response_endofpacket   => open
    );

  process (clk)
  begin
    if rising_edge(clk) then
      valid_r <= '0';
      if rst = '1' then
        cmd_valid <= '0';
      else
        -- Avalon-ST: hold command_valid until the core accepts it
        if start = '1' then
          cmd_valid <= '1';
        elsif cmd_ready = '1' then
          cmd_valid <= '0';
        end if;

        if rsp_valid = '1' then
          data_r  <= rsp_data;
          valid_r <= '1';
        end if;
      end if;
    end if;
  end process;

  data  <= data_r;
  valid <= valid_r;
end architecture rtl;
