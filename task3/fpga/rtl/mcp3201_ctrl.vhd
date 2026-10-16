-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : mcp3201_ctrl.vhd
-- Purpose : SPI master for the Microchip MCP3201 12-bit ADC.
--
-- MCP3201 read sequence (SPI mode 0,0 : SCLK idles low):
--   CS falls -> clocks 1..1.5 = sample window, the input is sampled
--   on the falling edge of clock 2 the ADC outputs a NULL bit, then
--   B11..B0 on the following falling edges.  We shift MISO in on every
--   rising edge for 15 clocks; the last 12 bits shifted in are B11..B0.
--
--          tick
--   CS_N   ---\______________________________________________/----
--   SCLK   ______/-\_/-\_/-\_/-\_ ... _/-\_/-\________________
--   DOUT   -----------------<0 ><B11><B10> ... <B0>---------------
--                    1   2   3   4         14  15
--
-- One conversion = 0.5 + 15 SCLK periods (15.5 us @ 1 MHz) << 62.5 us.
-- The conversion is started by 'start' (the hardware sample tick), so the
-- analog sampling instant is a fixed number of 50 MHz cycles after the
-- tick -> zero sampling jitter (only the crystal's own jitter remains).
--
-- SCLK limit: fCLK(max) = 1.6 MHz @ 5 V, 0.8 MHz @ 2.7 V.  At 3.3 V use
-- <= 1 MHz (default).  CLK_HZ/(2*SCLK_HZ) must be an integer >= 2.
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mcp3201_ctrl is
  generic (
    CLK_HZ  : positive := 50_000_000;
    SCLK_HZ : positive := 1_000_000
  );
  port (
    clk      : in  std_logic;
    rst      : in  std_logic;
    start    : in  std_logic;                      -- 1-clock start pulse
    -- ADC pins
    adc_cs_n : out std_logic;
    adc_sclk : out std_logic;
    adc_miso : in  std_logic;                      -- MCP3201 DOUT
    -- result
    data     : out std_logic_vector(11 downto 0);
    valid    : out std_logic;                      -- 1-clock pulse
    busy     : out std_logic
  );
end entity mcp3201_ctrl;

architecture rtl of mcp3201_ctrl is
  constant HALF   : positive := CLK_HZ / (2 * SCLK_HZ);   -- 25 @ 1 MHz
  constant NCLK   : positive := 15;

  type state_t is (S_IDLE, S_CS_SETUP, S_CLK_HIGH, S_CLK_LOW, S_DONE);
  signal state    : state_t := S_IDLE;
  signal div      : natural range 0 to HALF - 1 := 0;
  signal bitc     : natural range 0 to NCLK := 0;
  signal shreg    : std_logic_vector(NCLK - 1 downto 0) := (others => '0');
  signal miso_s   : std_logic_vector(1 downto 0) := (others => '0');
  signal cs_n_r   : std_logic := '1';
  signal sclk_r   : std_logic := '0';
  signal data_r   : std_logic_vector(11 downto 0) := (others => '0');
  signal valid_r  : std_logic := '0';
begin
  assert HALF >= 2
    report "mcp3201_ctrl: SCLK_HZ too high for CLK_HZ" severity failure;

  process (clk)
  begin
    if rising_edge(clk) then
      -- 2-FF synchroniser for the asynchronous ADC output
      miso_s <= miso_s(0) & adc_miso;

      valid_r <= '0';

      if rst = '1' then
        state  <= S_IDLE;
        cs_n_r <= '1';
        sclk_r <= '0';
        div    <= 0;
        bitc   <= 0;
      else
        case state is
          when S_IDLE =>
            cs_n_r <= '1';
            sclk_r <= '0';
            div    <= 0;
            bitc   <= 0;
            if start = '1' then
              cs_n_r <= '0';                         -- start conversion
              state  <= S_CS_SETUP;
            end if;

          when S_CS_SETUP =>                         -- tSUCS (>=100 ns)
            if div = HALF - 1 then
              div    <= 0;
              sclk_r <= '1';                         -- rising edge #1
              shreg  <= shreg(NCLK - 2 downto 0) & miso_s(1);
              bitc   <= 1;
              state  <= S_CLK_HIGH;
            else
              div <= div + 1;
            end if;

          when S_CLK_HIGH =>
            if div = HALF - 1 then
              div    <= 0;
              sclk_r <= '0';                         -- falling edge: ADC shifts
              state  <= S_CLK_LOW;
            else
              div <= div + 1;
            end if;

          when S_CLK_LOW =>
            if div = HALF - 1 then
              div <= 0;
              if bitc = NCLK then
                cs_n_r <= '1';                       -- end of conversion
                state  <= S_DONE;
              else
                sclk_r <= '1';                       -- rising edge #bitc+1
                shreg  <= shreg(NCLK - 2 downto 0) & miso_s(1);
                bitc   <= bitc + 1;
                state  <= S_CLK_HIGH;
              end if;
            else
              div <= div + 1;
            end if;

          when S_DONE =>
            data_r  <= shreg(11 downto 0);           -- B11..B0
            valid_r <= '1';
            state   <= S_IDLE;
        end case;
      end if;
    end if;
  end process;

  adc_cs_n <= cs_n_r;
  adc_sclk <= sclk_r;
  data     <= data_r;
  valid    <= valid_r;
  busy     <= '0' when state = S_IDLE else '1';
end architecture rtl;
