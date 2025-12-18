----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 11/17/2025 12:45:20 PM
-- Design Name: 
-- Module Name: TX_CONFIGURABLE_SERIAL - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity TX_CONFIGURABLE_SERIAL is
    Port (
        Clk   : in  STD_LOGIC;
        Reset : in  STD_LOGIC;
        Start : in  STD_LOGIC;
        Data  : in  STD_LOGIC_VECTOR (8 downto 0);
        baud_sel  : in std_logic_vector(5 downto 0); -- configurable to 36 standard bps
        stop_bit  : in std_logic_vector(2 downto 0);  -- 1 (010), 1.5(011) or 2(100) stop bits ( 2, 3 or 4 half ticks)
        parity    : in std_logic_vector(2 downto 0);  -- 0→Even, 1→Odd, 2→Mark(=1), 3→Space(=0), 4→parity disabled
        bit_order : in std_logic;                     -- 0→LSB-first (default), 1→MSB-first
        data_bits  : in std_logic_vector(2 downto 0); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b                
        EOT   : out STD_LOGIC;
        TX    : out STD_LOGIC
    );
end TX_CONFIGURABLE_SERIAL;

architecture Behavioral of TX_CONFIGURABLE_SERIAL is

  component NCO
    generic(
    N : integer := 32
  );
    port(
    clk       : in  std_logic;
    rst       : in  std_logic;                 -- activo a '0'
    en        : in  std_logic;
    half_mode : in  std_logic;                 -- '0' -> inc normal, '1' -> inc_half (2x baud)
    baud_sel  : in  std_logic_vector(5 downto 0); -- valor decimal del baud
    tick      : out std_logic
  );
  end component;

    -- FSM
    --type estados is (Idle, StartBit, SendData, ParityBit, StopBit);
    type estados is (Idle, StartBit, SendData, StopBit, ParityBit);
    
    signal current_state, next_state: estados;

    -- counters
    signal data_count_reg, data_count_next  : std_logic_vector(3 downto 0); -- 4 bits to count 0..9 databits
    signal stop_bits_reg, stop_bits_next    : std_logic_vector(2 downto 0); --  1 (010), 1.5(011) or 2(100) stop bits ( 2, 3 or 4 half ticks)
    --signal data_count : std_logic_vector(3 downto 0); -- LSB for MUX 1..9
    -- data_bits 0..4 to 5..9
    signal data_bits_5_to_9 : std_logic_vector(3 downto 0);
    -- NCO ticks
    signal NCO_tick : std_logic;
    -- control signals NCO, data_count, stop_bits counters
    signal en_NCO, rst_NCO, rst_NCO_mux, half_mode_NCO : std_logic;
    signal en_data_count, rst_data_count, en_stop_bits, rst_stop_bits : std_logic;
    -- outputs intermedios
    signal EOT_temp, TX_temp : std_logic := '1';
    signal TX_MUX, TX_MUX_LSB, TX_MUX_MSB, TX_MUX_MSB_5, TX_MUX_MSB_6, TX_MUX_MSB_7, TX_MUX_MSB_8, TX_MUX_MSB_9 : std_logic;
    signal TX_MUX_PARITY, PARITY_RAW  : std_logic;
begin
    
    rst_NCO_mux <= '0' when (Reset = '0') else rst_NCO;      
     
    NCO_component : NCO 
    port map (
        clk       => Clk,
        rst       => rst_NCO_mux,
        en        => en_NCO,
        half_mode => half_mode_NCO,
        baud_sel  => baud_sel,
        tick      => NCO_tick);
                               
    -- Data_count and stop_bits reg
    process(Clk, Reset)
    begin
        if Reset = '0' then
            data_count_reg <= (others => '0');
            stop_bits_reg <= (others => '0');            
        elsif rising_edge(Clk) then
            if rst_data_count = '1' then
                data_count_reg <= (others => '0');
            elsif en_data_count = '1' then
                data_count_reg <= data_count_next;
            end if;
            if rst_stop_bits = '1' then 
                stop_bits_reg <= (others => '0');
            elsif en_stop_bits = '1' then
                stop_bits_reg <= stop_bits_next;
            end if;
        end if;
    end process;

    data_count_next <= std_logic_vector(unsigned(data_count_reg) + 1);
    stop_bits_next <= std_logic_vector(unsigned(stop_bits_reg) + 1);
    
    -- MUX TX (LSB first)
    with data_count_reg select
    TX_MUX_LSB <= Data(0) when "0000", Data(1) when "0001", Data(2) when "0010", Data(3) when "0011", Data(4) when "0100", Data(5) when "0101", Data(6) when "0110", Data(7) when "0111", Data(8) when others;
                  
    -- MUX TX (MSB-first) dependiente del tamaño real de palabra
    with data_count_reg select TX_MUX_MSB_5 <= Data(4) when "0000", Data(3) when "0001", Data(2) when "0010", Data(1) when "0011", Data(0) when others;
    with data_count_reg select TX_MUX_MSB_6 <= Data(5) when "0000", Data(4) when "0001", Data(3) when "0010", Data(2) when "0011", Data(1) when "0100", Data(0) when others;
    with data_count_reg select TX_MUX_MSB_7 <= Data(6) when "0000", Data(5) when "0001", Data(4) when "0010", Data(3) when "0011", Data(2) when "0100", Data(1) when "0101", Data(0) when others;
    with data_count_reg select TX_MUX_MSB_8 <= Data(7) when "0000", Data(6) when "0001", Data(5) when "0010", Data(4) when "0011", Data(3) when "0100", Data(2) when "0101", Data(1) when "0110", Data(0) when others;
    with data_count_reg select TX_MUX_MSB_9 <= Data(8) when "0000", Data(7) when "0001", Data(6) when "0010", Data(5) when "0011", Data(4) when "0100", Data(3) when "0101", Data(2) when "0110", Data(1) when "0111", Data(0) when others;

    with data_bits select
        TX_MUX_MSB <= TX_MUX_MSB_5 when "000",
                  TX_MUX_MSB_6 when "001",
                  TX_MUX_MSB_7 when "010",
                  TX_MUX_MSB_8 when "011",
                  TX_MUX_MSB_9 when others;        
        
                
    -- MUX TX MUX
    TX_MUX <= TX_MUX_LSB when bit_order = '0' else TX_MUX_MSB;
    
    -- data_bits 0..4 to 5..9
    with data_bits select
        data_bits_5_to_9 <= "0101" when "000",
                            "0110" when "001",
                            "0111" when "010",
                            "1000" when "011",
                            "1001" when others;
                        
    -- PARITY MUX
    PARITY_RAW <=
        -- 5 bits
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4)) when data_bits="000" else
        -- 6 bits
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5)) when data_bits="001" else
        -- 7 bits
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6)) when data_bits="010" else
        -- 8 bits
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6) xor Data(7)) when data_bits="011" else
        -- 9 bits
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6) xor Data(7) xor Data(8));
        
    with parity select 
        TX_MUX_PARITY <=    PARITY_RAW  when "000",
                            not PARITY_RAW      when "001",
                            '1'             when "010",
                            '0'             when others;
                                                
    -- FSM combinational: calcula next_state y señales de salida temporales
    process(current_state, Start, Data, data_count_reg, stop_bits_reg, NCO_tick, data_bits_5_to_9, parity, stop_bit, TX_MUX)
    begin
        -- default assignments
        next_state <= current_state;
        EOT_temp <= '0';
        TX_temp <= '1';
        en_data_count <= '0';
        rst_data_count <= '0';
        en_stop_bits <= '0';
        rst_stop_bits <= '0';
        en_NCO <= '1';
        half_mode_NCO <= '0';
        rst_NCO <= '1';
        case current_state is
            when Idle =>
                EOT_temp <= '1';
                en_NCO <= '0';
                if Start = '1' then
                    next_state <= StartBit;
                    rst_data_count <= '1';  
                    rst_NCO <= '0';                                   
                end if;

            when StartBit =>
                TX_temp <= '0';  -- start bit = 0
                if NCO_tick = '1' then
                    next_state <= SendData;
                end if;

            when SendData =>
                en_data_count <= NCO_tick;
                TX_temp <= TX_MUX;
                -- comparar con data_count_reg (4 bits) a data_bits_5_to_9
                if (data_count_reg = data_bits_5_to_9) then
                en_data_count <= '0';  
                    if (parity = "100") then
                        next_state <= StopBit;
                        TX_temp <= '1'; -- stop bit is typically '1'
                        --rst_NCO <= '0';
                    else
                        next_state <= ParityBit;
                        TX_temp <= TX_MUX_PARITY;
                    end if;                                          
                end if;
            
            when ParityBit =>
                TX_temp <= TX_MUX_PARITY;
                if NCO_tick = '1' then
                    next_state <= StopBit;
                    TX_temp <= '1'; -- stop bit is typically '1'                
                end if;

            when StopBit =>
                en_stop_bits <= NCO_tick;
                half_mode_NCO <= '1';
                TX_temp <= '1';
                if stop_bits_reg = stop_bit then
                    next_state <= Idle;
                    EOT_temp <= '1';
                    rst_NCO <= '0';
                    rst_stop_bits <= '1';
                end if;

        end case;
    end process;

    -- State register (synchronous): actualiza current_state y genera resets de 1 ciclo
    process(Clk, Reset)
    begin
        if Reset = '0' then
            current_state <= Idle;
        elsif rising_edge(Clk) then
            -- update state
            current_state <= next_state;
        end if;
    end process;

    -- outputs
    TX <= TX_temp;
    EOT <= EOT_temp;

end Behavioral;
