----------------------------------------------------------------------------------
-- ShiftRegister - palabra normalizada en Q (independiente de data_bits y bit_order)
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ShiftRegister is
    Port (
      Reset     : in  STD_LOGIC;
      Clk       : in  STD_LOGIC;
      Enable    : in  STD_LOGIC;                 -- pulso por bit recibido
      data_bits : in  std_logic_vector(2 downto 0); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b   
      bit_order : in  std_logic;                 -- '0' = LSB-first, '1' = MSB-first
      D         : in  STD_LOGIC;                 -- bit muestreado (en el centro)
      Q         : out STD_LOGIC_VECTOR (8 downto 0)  -- palabra NORMALIZADA (LSB en Q(0))
    );
end ShiftRegister;

architecture Behavioral of ShiftRegister is
  signal d_reg, d_next, d_lsb, d_msb: std_logic_vector(8 downto 0);
begin
-- reg
  process (Reset, Clk)
  begin
    if Reset = '0' then
      d_reg <= (others => '0');
    elsif rising_edge(Clk) and Enable = '1' then
        d_reg <= d_next;      
    end if;
  end process;
-- next state logic
                   
    with data_bits select
        d_lsb <=    D & d_reg(8 downto 1) when "100",
                   '0' & D & d_reg(7 downto 1) when "011",
                   "00" & D & d_reg(6 downto 1) when "010",
                   "000" & D & d_reg(5 downto 1) when "001",
                   "0000" & D & d_reg(4 downto 1) when others;
                   
    with data_bits select         
        d_msb <=  d_reg(7 downto 0) & D when "100",
                   '0' & d_reg(6 downto 0) & D when "011",
                   "00" & d_reg(5 downto 0) & D when "010",
                   "000" & d_reg(4 downto 0) & D when "001",
                   "0000" & d_reg(3 downto 0) & D when others;   
                      
    with bit_order select
        d_next <= d_lsb when '0', d_msb when others;                                
-- output logic
    Q <= d_reg;
end Behavioral;
