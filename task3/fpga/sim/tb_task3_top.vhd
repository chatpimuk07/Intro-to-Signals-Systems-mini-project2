-- =========================================================================
-- Signals & Systems / DSP Practical Labs - Task 3 (v2, VHDL)
-- File    : tb_task3_top.vhd
-- Purpose : Self-checking testbench for the complete FPGA design.
--   * behavioural MCP3201 model returns a known code sequence
--       code(n) = (37*n + 100) mod 4096   for conversion n = 0,1,2,...
--   * behavioural ESP32 SPI master (mode 0, 4 MHz) waits for
--     FPGA_DATA_READY and reads complete 2056-byte frames
--   * checks: magic, N, sequence numbers, every sample value,
--             sampling period on TIMING_DEBUG_PIN, test-tone mode status
--
-- Fs is raised to 50 kS/s in simulation (generic) only to shorten the run
-- time - the logic is identical to the 16 kS/s build.
-- Run:  sim/run_sim.sh   (nvc or GHDL)
-- =========================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_task3_top is
end entity tb_task3_top;

architecture sim of tb_task3_top is
  constant CLK_HZ    : positive := 50_000_000;
  constant FS_HZ     : positive := 50_000;
  constant ADDR_BITS : positive := 10;
  constant N         : positive := 2 ** ADDR_BITS;
  constant FRAME_B   : positive := 8 + 2 * N;
  constant TCLK      : time := 20 ns;
  constant TS        : time := 1 sec / FS_HZ;
  constant SPI_HALF  : time := 125 ns;                -- 4 MHz SCK

  signal clk      : std_logic := '0';
  signal key      : std_logic_vector(1 downto 0) := "10";   -- KEY0 pressed = reset
  signal sw       : std_logic_vector(9 downto 0) := (others => '0');
  signal ledr     : std_logic_vector(9 downto 0);
  signal adc_cs_n : std_logic;
  signal adc_sck  : std_logic;
  signal adc_miso : std_logic := 'Z';
  signal esp_sck  : std_logic := '0';
  signal esp_cs_n : std_logic := '1';
  signal esp_miso : std_logic;
  signal ready    : std_logic;
  signal tdbg     : std_logic;
  signal done     : boolean := false;

  function adc_code(n : natural) return natural is
  begin
    return (37 * n + 100) mod 4096;
  end function;

  type byte_arr is array (0 to FRAME_B - 1) of std_logic_vector(7 downto 0);

  function w16(hi, lo : std_logic_vector(7 downto 0)) return natural is
  begin
    return 256 * to_integer(unsigned(hi)) + to_integer(unsigned(lo));
  end function;
begin
  clk <= not clk after TCLK / 2 when not done else '0';

  dut : entity work.task3_top
    generic map (CLK_HZ => CLK_HZ, FS_HZ => FS_HZ, ADC_SCLK_HZ => 1_000_000,
                 ADDR_BITS => ADDR_BITS, USE_MAX10_ADC => false)
    port map (
      MAX10_CLK1_50 => clk, KEY => key, SW => sw, LEDR => ledr,
      ADC_SPI_CS_N => adc_cs_n, ADC_SPI_SCK => adc_sck, ADC_SPI_MISO => adc_miso,
      ESP_SPI_SCK => esp_sck, ESP_SPI_CS_N => esp_cs_n, ESP_SPI_MISO => esp_miso,
      FPGA_DATA_READY => ready, TIMING_DEBUG_PIN => tdbg);

  ---------------------------------------------------------------------------
  -- MCP3201 behavioural model (DOUT changes on SCLK falling edge, tDO=100 ns)
  ---------------------------------------------------------------------------
  mcp_model : process
    variable conv : natural := 0;
    variable code : std_logic_vector(11 downto 0);
    variable nfall : natural;
  begin
    adc_miso <= 'Z';
    wait until falling_edge(adc_cs_n);
    loop
      code  := std_logic_vector(to_unsigned(adc_code(conv), 12));
      nfall := 0;
      adc_miso <= 'Z';
      while adc_cs_n = '0' loop
        wait until falling_edge(adc_sck) or rising_edge(adc_cs_n);
        exit when adc_cs_n = '1';
        nfall := nfall + 1;
        if nfall = 2 then
          adc_miso <= '0' after 100 ns;                     -- null bit
        elsif nfall >= 3 and nfall <= 14 then
          adc_miso <= code(14 - nfall) after 100 ns;        -- B11..B0
        else
          adc_miso <= 'X' after 100 ns;                     -- LSB-first data: ignored
        end if;
      end loop;
      adc_miso <= 'Z';
      conv := conv + 1;
      wait until falling_edge(adc_cs_n);
    end loop;
  end process;

  ---------------------------------------------------------------------------
  -- sampling period check on TIMING_DEBUG_PIN (toggles once per sample)
  ---------------------------------------------------------------------------
  period_chk : process
    variable t_last : time := 0 ns;
    variable cnt    : natural := 0;
  begin
    wait until rising_edge(tdbg);
    t_last := now;
    while not done loop
      wait until tdbg'event or done;
      exit when done;
      assert (now - t_last) = TS
        report "sample period error: " & time'image(now - t_last) severity error;
      t_last := now;
      cnt := cnt + 1;
    end loop;
    report "period check: " & integer'image(cnt) & " sample intervals of "
           & time'image(TS) & " verified";
    wait;
  end process;

  ---------------------------------------------------------------------------
  -- ESP32 master
  ---------------------------------------------------------------------------
  esp32 : process
    variable rx     : byte_arr;
    variable b      : std_logic_vector(7 downto 0);
    variable seq    : natural;
    variable stat   : std_logic_vector(15 downto 0);
    variable smp    : natural;
    variable errors : natural := 0;
    variable vmin, vmax : natural;

    procedure read_frame is
    begin
      esp_cs_n <= '0';
      wait for 1 us;
      for i in 0 to FRAME_B - 1 loop
        for k in 7 downto 0 loop
          wait for SPI_HALF;
          esp_sck <= '1';
          b(k) := esp_miso;                  -- sample on rising edge
          wait for SPI_HALF;
          esp_sck <= '0';
        end loop;
        rx(i) := b;
      end loop;
      wait for 1 us;
      esp_cs_n <= '1';
      wait for 1 us;
    end procedure;

    procedure check_header(exp_seq : natural) is
    begin
      assert rx(0) = x"A5" and rx(1) = x"5A" report "bad magic" severity error;
      assert w16(rx(4), rx(5)) = N report "bad N" severity error;
      seq  := w16(rx(2), rx(3));
      stat := std_logic_vector(to_unsigned(w16(rx(6), rx(7)), 16));
      assert seq = exp_seq
        report "seq " & integer'image(seq) & " expected " & integer'image(exp_seq)
        severity error;
    end procedure;
  begin
    wait for 1 us;
    key(0) <= '1';                                   -- release reset

    -- ---- ADC mode : read 3 consecutive blocks, verify every sample ----
    for blk in 0 to 2 loop
      wait until ready = '1';
      wait for 3 us;
      read_frame;
      check_header(blk);
      assert stat(15) = '0' report "status: expected ADC mode" severity error;
      assert stat(7 downto 0) = x"00" report "unexpected overrun" severity error;
      for i in 0 to N - 1 loop
        smp := w16(rx(8 + 2 * i), rx(9 + 2 * i));
        if smp /= adc_code(blk * N + i) then
          if errors < 10 then
            report "block " & integer'image(blk) & " sample " & integer'image(i) &
                   " = " & integer'image(smp) & " expected " &
                   integer'image(adc_code(blk * N + i)) severity error;
          end if;
          errors := errors + 1;
        end if;
      end loop;
      assert ready = '0' report "DATA_READY not cleared after frame" severity error;
      report "block " & integer'image(blk) & " checked";
    end loop;

    -- ---- skip one block on purpose -> overrun counter must count it ----
    wait until ready = '1';                          -- block 3 ready ... not read
    wait for (N + 10) * TS;                          -- block 4 completes -> overrun
    read_frame;
    check_header(4);
    assert stat(7 downto 0) = x"01" report "overrun counter should be 1" severity error;
    report "overrun detection checked";

    -- ---- test-tone mode (SW0=1, SW[3:1]=000 -> 1 kHz) ----
    sw(0) <= '1';
    wait until ready = '1';                          -- partially mixed block
    read_frame;
    wait until ready = '1';
    read_frame;
    seq := w16(rx(2), rx(3));
    stat := std_logic_vector(to_unsigned(w16(rx(6), rx(7)), 16));
    assert stat(15) = '1' report "status: expected test-tone mode" severity error;
    vmin := 4095; vmax := 0;
    for i in 0 to N - 1 loop
      smp := w16(rx(8 + 2 * i), rx(9 + 2 * i));
      if smp < vmin then vmin := smp; end if;
      if smp > vmax then vmax := smp; end if;
    end loop;
    report "test tone block min=" & integer'image(vmin) & " max=" & integer'image(vmax);
    assert vmin >= 2048 - 1800 and vmin < 400 and vmax <= 2048 + 1800 and vmax > 3700
      report "test tone amplitude out of range" severity error;

    if errors = 0 then
      report "*** TESTBENCH PASSED ***";
    else
      report "*** TESTBENCH FAILED: " & integer'image(errors) & " sample errors ***"
        severity error;
    end if;
    done <= true;
    wait;
  end process;
end architecture sim;
