----------------------------------------------------------------------------------
-- NCO Optimizado con Selector de Baudios (Indexado)
-- Entrada: 6 bits (0..63) en lugar de 22 bits.
-- Ahorro de recursos: 16 pines de entrada menos.
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity NCO is
  generic(
    N : integer := 32
  );
  port(
    clk       : in  std_logic;
    rst       : in  std_logic;
    en        : in  std_logic;
    half_mode : in  std_logic;                   -- '0'=Normal, '1'=Doble Velocidad
    baud_sel  : in  std_logic_vector(5 downto 0); -- Selector (0 a 53)
    tick      : out std_logic
  );
end NCO;

architecture Behavioral of NCO is

  -- Constante de frecuencia del sistema (Ajustar si cambia el reloj)
  constant SYSTEM_CLK_FREQ : integer := 100_000_000; 

  -- Función de cálculo de incremento (Synthesis-time)
  function calc_inc(baud_rate : integer; clk_freq : integer) return unsigned is
    variable numerator : unsigned(63 downto 0);
    variable result    : unsigned(63 downto 0);
  begin
    numerator := to_unsigned(baud_rate, 64);
    numerator := numerator sll 32;
    result    := numerator / to_unsigned(clk_freq, 64);
    return result(31 downto 0);
  end function;

  -- Señales internas
  signal phase_reg, phase_next : unsigned(N-1 downto 0);
  signal sum                   : unsigned(N downto 0);
  signal inc_rom, inc_half_rom : unsigned(N-1 downto 0);
  signal inc_i                 : unsigned(N-1 downto 0);
  signal tick_reg, tick_next   : std_logic;

begin

  ----------------------------------------------------------------------------
  -- DECODIFICADOR DE BAUDIOS (LUT)
  ----------------------------------------------------------------------------
  rom_proc : process(baud_sel)
    variable sel : integer;
  begin
    sel := to_integer(unsigned(baud_sel));
    
    -- Valores por defecto (Reset/Desconocido)
    inc_rom      <= (others => '0');
    inc_half_rom <= (others => '0');

    case sel is
      -- === BAJAS FRECUENCIAS ===
      when 0  => inc_rom <= calc_inc(50, SYSTEM_CLK_FREQ);    inc_half_rom <= calc_inc(100, SYSTEM_CLK_FREQ);
      when 1  => inc_rom <= calc_inc(75, SYSTEM_CLK_FREQ);    inc_half_rom <= calc_inc(150, SYSTEM_CLK_FREQ);
      when 2  => inc_rom <= calc_inc(110, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(220, SYSTEM_CLK_FREQ);
      when 3  => inc_rom <= calc_inc(134, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(269, SYSTEM_CLK_FREQ);
      when 4  => inc_rom <= calc_inc(150, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(300, SYSTEM_CLK_FREQ);
      when 5  => inc_rom <= calc_inc(200, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(400, SYSTEM_CLK_FREQ);
      when 6  => inc_rom <= calc_inc(300, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(600, SYSTEM_CLK_FREQ);
      when 7  => inc_rom <= calc_inc(600, SYSTEM_CLK_FREQ);   inc_half_rom <= calc_inc(1200, SYSTEM_CLK_FREQ);
      when 8  => inc_rom <= calc_inc(1200, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(2400, SYSTEM_CLK_FREQ);
      when 9  => inc_rom <= calc_inc(1800, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(3600, SYSTEM_CLK_FREQ);
      when 10 => inc_rom <= calc_inc(2000, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(4000, SYSTEM_CLK_FREQ);
      when 11 => inc_rom <= calc_inc(2400, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(4800, SYSTEM_CLK_FREQ);
      when 12 => inc_rom <= calc_inc(3600, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(7200, SYSTEM_CLK_FREQ);
      when 13 => inc_rom <= calc_inc(4800, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(9600, SYSTEM_CLK_FREQ);
      when 14 => inc_rom <= calc_inc(7200, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(14400, SYSTEM_CLK_FREQ);
      
      -- === ESTÁNDARES COMUNES ===
      when 15 => inc_rom <= calc_inc(9600, SYSTEM_CLK_FREQ);  inc_half_rom <= calc_inc(19200, SYSTEM_CLK_FREQ);
      when 16 => inc_rom <= calc_inc(12000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(24000, SYSTEM_CLK_FREQ);
      when 17 => inc_rom <= calc_inc(14400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(28800, SYSTEM_CLK_FREQ);
      when 18 => inc_rom <= calc_inc(19200, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(38400, SYSTEM_CLK_FREQ);
      when 19 => inc_rom <= calc_inc(28800, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(57600, SYSTEM_CLK_FREQ);
      
      -- === PROTOCOLOS ESPECIALES ===
      when 20 => inc_rom <= calc_inc(31250, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(62500, SYSTEM_CLK_FREQ); -- MIDI
      when 21 => inc_rom <= calc_inc(38400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(76800, SYSTEM_CLK_FREQ);
      when 22 => inc_rom <= calc_inc(50000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(100000, SYSTEM_CLK_FREQ);
      when 23 => inc_rom <= calc_inc(56000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(112000, SYSTEM_CLK_FREQ);
      when 24 => inc_rom <= calc_inc(57600, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(115200, SYSTEM_CLK_FREQ);
      
      -- === ALTA VELOCIDAD ===
      when 25 => inc_rom <= calc_inc(64000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(128000, SYSTEM_CLK_FREQ);
      when 26 => inc_rom <= calc_inc(74400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(148800, SYSTEM_CLK_FREQ);
      when 27 => inc_rom <= calc_inc(74880, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(149760, SYSTEM_CLK_FREQ);
      when 28 => inc_rom <= calc_inc(76800, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(153600, SYSTEM_CLK_FREQ);
      
      when 29 => inc_rom <= calc_inc(115200, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(230400, SYSTEM_CLK_FREQ); -- STD
      when 30 => inc_rom <= calc_inc(128000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(256000, SYSTEM_CLK_FREQ);
      when 31 => inc_rom <= calc_inc(153600, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(307200, SYSTEM_CLK_FREQ);
      when 32 => inc_rom <= calc_inc(200000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(400000, SYSTEM_CLK_FREQ);
      when 33 => inc_rom <= calc_inc(230400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(460800, SYSTEM_CLK_FREQ);
      when 34 => inc_rom <= calc_inc(250000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(500000, SYSTEM_CLK_FREQ); -- DMX
      when 35 => inc_rom <= calc_inc(256000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(512000, SYSTEM_CLK_FREQ);
      when 36 => inc_rom <= calc_inc(312500, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(625000, SYSTEM_CLK_FREQ);
      when 37 => inc_rom <= calc_inc(400000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(800000, SYSTEM_CLK_FREQ);
      when 38 => inc_rom <= calc_inc(460800, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(921600, SYSTEM_CLK_FREQ);
      when 39 => inc_rom <= calc_inc(500000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(1000000, SYSTEM_CLK_FREQ);
      when 40 => inc_rom <= calc_inc(576000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(1152000, SYSTEM_CLK_FREQ);
      when 41 => inc_rom <= calc_inc(614400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(1228800, SYSTEM_CLK_FREQ);
      when 42 => inc_rom <= calc_inc(750000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(1500000, SYSTEM_CLK_FREQ);
      when 43 => inc_rom <= calc_inc(921600, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(1843200, SYSTEM_CLK_FREQ);
      
      -- === MEGABAUDIOS ===
      when 44 => inc_rom <= calc_inc(1000000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(2000000, SYSTEM_CLK_FREQ);
      when 45 => inc_rom <= calc_inc(1152000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(2304000, SYSTEM_CLK_FREQ);
      when 46 => inc_rom <= calc_inc(1500000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(3000000, SYSTEM_CLK_FREQ);
      when 47 => inc_rom <= calc_inc(1843200, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(3686400, SYSTEM_CLK_FREQ);
      when 48 => inc_rom <= calc_inc(2000000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(4000000, SYSTEM_CLK_FREQ);
      when 49 => inc_rom <= calc_inc(2500000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(5000000, SYSTEM_CLK_FREQ);
      when 50 => inc_rom <= calc_inc(3000000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(6000000, SYSTEM_CLK_FREQ);
      when 51 => inc_rom <= calc_inc(3500000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(7000000, SYSTEM_CLK_FREQ);
      when 52 => inc_rom <= calc_inc(3686400, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(7372800, SYSTEM_CLK_FREQ);
      when 53 => inc_rom <= calc_inc(4000000, SYSTEM_CLK_FREQ); inc_half_rom <= calc_inc(8000000, SYSTEM_CLK_FREQ);

      when others => null; -- Reservado para el futuro
    end case;
  end process;

  -- Multiplexor de salida (Normal vs Doble)
  inc_i <= inc_half_rom when (half_mode = '1') else inc_rom;

  -- Acumulador de Fase
  reg_proc : process(clk, rst)
  begin
    if rst = '0' then
      phase_reg <= (others => '0');
      tick_reg  <= '0';
    elsif rising_edge(clk) then
      if en = '1' then
        phase_reg <= phase_next;
        tick_reg  <= tick_next;
      end if;
    end if;
  end process;

  sum        <= ('0' & phase_reg) + ('0' & inc_i);
  phase_next <= sum(N-1 downto 0);
  tick_next  <= sum(N);
  tick       <= tick_reg;

end Behavioral;