----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 25.09.2025 18:10:31
-- Design Name: 
-- Module Name: RS232_RX - Behavioral
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

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity RX_CONFIGURABLE_SERIAL is
    Port (  Reset : in STD_LOGIC;
            Clk : in STD_LOGIC;
            baud_sel  : in std_logic_vector(5 downto 0); -- configurable to 36 standard bps
            stop_bit  : in std_logic_vector(2 downto 0);  -- 1 (010), 1.5(011) or 2(100) stop bits ( 2, 3 or 4 half ticks)
            parity    : in std_logic_vector(2 downto 0);  -- 0→Even, 1→Odd, 2→Mark(=1), 3→Space(=0), 4→parity disabled
            bit_order : in std_logic;                     -- 0→LSB-first (default), 1→MSB-first
            data_bits  : in std_logic_vector(2 downto 0); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b                           
            LineRD_in : in STD_LOGIC;
            shiftRegister_Q : in std_logic_vector(8 downto 0);
            PAR_ERROR : out std_logic;
            FRAME_ERROR : out std_logic;
            ERROR_OK : in std_logic;
            Valid_out : out STD_LOGIC;
            Code_out : out STD_LOGIC;
            Store_out : out STD_LOGIC);
end RX_CONFIGURABLE_SERIAL;

architecture Behavioral of RX_CONFIGURABLE_SERIAL is

component NCO
    generic(
    N : integer := 32
  );
    port(
    clk       : in  std_logic;
    rst       : in  std_logic;                 -- activo a '0'
    en        : in  std_logic;
    half_mode : in  std_logic;                 -- '0' -> inc normal, '1' -> inc_half (2x baud)
    baud_sel  : in  std_logic_vector(5 downto 0); 
    tick      : out std_logic
  );
  end component;



-- FSM
    type estados is (Idle, StartBit, RcvData, ParityBit, StopBit);
    signal current_state, next_state: estados := Idle;

    -- counters
    signal data_count_reg, data_count_next  : std_logic_vector(3 downto 0); -- 4 bits to count 0..9 databits
    --signal data_count : std_logic_vector(3 downto 0); -- LSB for MUX 1..9
    -- data_bits 0..4 to 5..9
    signal data_bits_5_to_9 : std_logic_vector(3 downto 0);
    -- NCO ticks
    signal NCO_tick : std_logic;
    -- control signals NCO, data_count, stop_bits counters
    signal en_NCO, rst_NCO, rst_NCO_mux, half_mode_NCO : std_logic;
    signal en_data_count, rst_data_count: std_logic;
  
    -- outputs intermedios
    signal Store_out_temp, Valid_out_temp, Code_out_temp : std_logic := '0';
    -- error flag
    signal par_error_reg, frame_error_reg, par_error_next, frame_error_next  : std_logic;
    -- parity check
    signal EXPECTED_PARITY, PARITY_RAW  : std_logic;

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

    -- outputs
    Code_out_temp <= LineRD_in;
    Code_out <= Code_out_temp;
    Valid_out <= Valid_out_temp;
    Store_out <= Store_out_temp;
    
    -- regs
    process(Clk, Reset)
    begin
        if Reset = '0' then
            data_count_reg <= (others => '0');
            par_error_reg <= '0';
            frame_error_reg <= '0';
        elsif ERROR_OK = '1' then
            par_error_reg <= '0';
            frame_error_reg <= '0';            
        elsif rising_edge(Clk) then
            if rst_data_count = '1' then
                data_count_reg <= (others => '0');
            elsif en_data_count = '1' then
                data_count_reg <= data_count_next;
            end if;
            par_error_reg <= par_error_next;
            frame_error_reg <= frame_error_next;
        end if;
    end process;

    -- data_count next logic (combinational)
    data_count_next <= std_logic_vector(unsigned(data_count_reg) + 1);

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
        (shiftRegister_Q(0) xor shiftRegister_Q(1) xor shiftRegister_Q(2) xor shiftRegister_Q(3) xor shiftRegister_Q(4)) when data_bits="000" else
        -- 6 bits
        (shiftRegister_Q(0) xor shiftRegister_Q(1) xor shiftRegister_Q(2) xor shiftRegister_Q(3) xor shiftRegister_Q(4) xor shiftRegister_Q(5)) when data_bits="001" else
        -- 7 bits
        (shiftRegister_Q(0) xor shiftRegister_Q(1) xor shiftRegister_Q(2) xor shiftRegister_Q(3) xor shiftRegister_Q(4) xor shiftRegister_Q(5) xor shiftRegister_Q(6)) when data_bits="010" else
        -- 8 bits
        (shiftRegister_Q(0) xor shiftRegister_Q(1) xor shiftRegister_Q(2) xor shiftRegister_Q(3) xor shiftRegister_Q(4) xor shiftRegister_Q(5) xor shiftRegister_Q(6) xor shiftRegister_Q(7)) when data_bits="011" else
        -- 9 bits
        (shiftRegister_Q(0) xor shiftRegister_Q(1) xor shiftRegister_Q(2) xor shiftRegister_Q(3) xor shiftRegister_Q(4) xor shiftRegister_Q(5) xor shiftRegister_Q(6) xor shiftRegister_Q(7) xor shiftRegister_Q(8));
        
    with parity select 
        EXPECTED_PARITY <=    PARITY_RAW  when "000",
                            not PARITY_RAW      when "001",
                            '1'             when "010",
                            '0'             when others;


    -- FSM combinational: calcula next_state y señales de salida temporales
    process(current_state, LineRD_in, data_count_reg, NCO_tick, data_bits_5_to_9, EXPECTED_PARITY, par_error_reg, frame_error_reg)
    begin
        -- default assignments
        next_state <= current_state;
        Store_out_temp <= '0';
        Valid_out_temp <= '0';
        
        en_NCO <= '1';
        en_data_count <= '0';
        half_mode_NCO <= '0';
        
        rst_NCO <= '1';
        rst_data_count <= '0';
        
        par_error_next <= par_error_reg;
        frame_error_next <= frame_error_reg;
        case current_state is
            when Idle =>
                en_NCO <= '0';
                if LineRD_in = '0' then
                    next_state <= StartBit;    
                    rst_NCO <= '0'; 
                    rst_data_count <= '1';              
                end if;

            when StartBit =>
                half_mode_NCO <= '1';
                if NCO_tick = '1' then
                    next_state <= RcvData;
                end if;

            when RcvData =>
                Valid_out_temp <= NCO_tick;
                en_data_count <= NCO_tick;
                -- comparar con DataCount_reg (4 bits) a 8 (1000)
                if data_count_reg = data_bits_5_to_9 then
                    if (parity = "100") then
                        next_state <= StopBit;
                    else
                        next_state <= ParityBit;
                    end if;                                          
                end if;
                
            when ParityBit =>
            if NCO_tick = '1' then
                if LineRD_in = EXPECTED_PARITY then
                    next_state <= StopBit;
                else 
                    par_error_next <= '1';
                    next_state <= Idle;
                end if;                 
            end if;

            when StopBit =>
            if NCO_tick = '1' then
                    next_state <= Idle;
                    if LineRD_in = '1' then
                        Store_out_temp <= '1';
                    else 
                        frame_error_next <= '1';
                    end if;
                end if;
        end case;
    end process;
    -- ERROR_OK reset error regs
    
    -- State register (synchronous): actualiza current_state
    process(Clk, Reset)
    begin
        if Reset = '0' then
            current_state <= Idle;
        elsif rising_edge(Clk) then
            -- update state
            current_state <= next_state;
        end if;
    end process;

end Behavioral;
