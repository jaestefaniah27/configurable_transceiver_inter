----------------------------------------------------------------------------------
-- Module Name: TX_CONFIGURABLE_SERIAL - Behavioral
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity TX_CONFIGURABLE_SERIAL is
    Port (
        Clk       : in  STD_LOGIC;
        Reset     : in  STD_LOGIC;
        Start     : in  STD_LOGIC;
        Data      : in  STD_LOGIC_VECTOR (8 downto 0);
        baud_sel  : in std_logic_vector(5 downto 0);
        stop_bit  : in std_logic_vector(2 downto 0);
        parity    : in std_logic_vector(2 downto 0);
        bit_order : in std_logic;
        data_bits : in std_logic_vector(2 downto 0);                
        EOT       : out STD_LOGIC;
        DE        : out STD_LOGIC;
        TX        : out STD_LOGIC
    );
end TX_CONFIGURABLE_SERIAL;

architecture Behavioral of TX_CONFIGURABLE_SERIAL is

  component NCO
    generic(
    N : integer := 32
  );
    port(
    clk       : in  std_logic;
    rst       : in  std_logic;
    en        : in  std_logic;
    half_mode : in  std_logic;
    baud_sel  : in  std_logic_vector(5 downto 0);
    tick      : out std_logic
  );
  end component;

    -- FSM actualizadas con PreDE y PostDE
    type estados is (Idle, PreDE, StartBit, SendData, ParityBit, StopBit, PostDE);
    signal current_state, next_state: estados;

    -- Counters existentes
    signal data_count_reg, data_count_next  : std_logic_vector(3 downto 0);
    signal stop_bits_reg, stop_bits_next    : std_logic_vector(2 downto 0);
    
    -- Counter de Delay añadido para los 100us (10,000 ciclos a 100MHz)
    constant DELAY_CYCLES : integer := 10000;
    signal delay_cnt_reg, delay_cnt_next : unsigned(13 downto 0);
    signal en_delay, rst_delay : std_logic;

    signal data_bits_5_to_9 : std_logic_vector(3 downto 0);
    signal NCO_tick : std_logic;
    
    -- Control signals
    signal en_NCO, rst_NCO, rst_NCO_mux, half_mode_NCO : std_logic;
    signal en_data_count, rst_data_count, en_stop_bits, rst_stop_bits : std_logic;
    
    -- Outputs intermedios
    signal EOT_temp, TX_temp, DE_temp : std_logic;
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
                               
    -- Proceso síncrono para contadores
    process(Clk, Reset)
    begin
        if Reset = '0' then
            data_count_reg <= (others => '0');
            stop_bits_reg <= (others => '0');
            delay_cnt_reg <= (others => '0');            
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
            
            if rst_delay = '1' then
                delay_cnt_reg <= (others => '0');
            elsif en_delay = '1' then
                delay_cnt_reg <= delay_cnt_next;
            end if;
        end if;
    end process;

    data_count_next <= std_logic_vector(unsigned(data_count_reg) + 1);
    stop_bits_next <= std_logic_vector(unsigned(stop_bits_reg) + 1);
    delay_cnt_next <= delay_cnt_reg + 1;
    
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
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4)) when data_bits="000" else
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5)) when data_bits="001" else
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6)) when data_bits="010" else
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6) xor Data(7)) when data_bits="011" else
        (Data(0) xor Data(1) xor Data(2) xor Data(3) xor Data(4) xor Data(5) xor Data(6) xor Data(7) xor Data(8));
        
    with parity select 
        TX_MUX_PARITY <=    PARITY_RAW  when "000",
                            not PARITY_RAW      when "001",
                            '1'             when "010",
                            '0'             when others;
                                                
    -- FSM combinational
    process(current_state, Start, Data, data_count_reg, stop_bits_reg, delay_cnt_reg, NCO_tick, data_bits_5_to_9, parity, stop_bit, TX_MUX, TX_MUX_PARITY)
    begin
        -- Asignaciones por defecto
        next_state <= current_state;
        EOT_temp <= '0';
        DE_temp <= '0';
        TX_temp <= '1';
        en_data_count <= '0';
        rst_data_count <= '0';
        en_stop_bits <= '0';
        rst_stop_bits <= '0';
        en_NCO <= '1';
        half_mode_NCO <= '0';
        rst_NCO <= '1';
        en_delay <= '0';
        rst_delay <= '0';
        
        case current_state is
            when Idle =>
                EOT_temp <= '1';
                en_NCO <= '0';
                if Start = '1' then
                    next_state <= PreDE;
                    rst_delay <= '1';
                    DE_temp <= '1';                                  
                end if;

            when PreDE =>
                DE_temp <= '1';
                en_delay <= '1';
                en_NCO <= '0';
                -- Esperar (DELAY_CYCLES - 1) garantiza exactamente 10000 periodos al transicionar
                if delay_cnt_reg = to_unsigned(DELAY_CYCLES - 1, 14) then
                    next_state <= StartBit;
                    rst_data_count <= '1';
                    rst_NCO <= '0'; -- Reinicia el NCO justo antes de transmitir el StartBit
                end if;

            when StartBit =>
                TX_temp <= '0'; 
                DE_temp <= '1';
                if NCO_tick = '1' then
                    next_state <= SendData;
                end if;

            when SendData =>
                en_data_count <= NCO_tick;
                TX_temp <= TX_MUX;
                DE_temp <= '1';
                if (data_count_reg = data_bits_5_to_9) then
                    en_data_count <= '0';  
                    if (parity = "100") then
                        next_state <= StopBit;
                        TX_temp <= '1'; 
                    else
                        next_state <= ParityBit;
                        TX_temp <= TX_MUX_PARITY;
                    end if;                                          
                end if;
            
            when ParityBit =>
                TX_temp <= TX_MUX_PARITY;
                DE_temp <= '1';
                if NCO_tick = '1' then
                    next_state <= StopBit;
                    TX_temp <= '1';                 
                end if;

            when StopBit =>
                en_stop_bits <= NCO_tick;
                half_mode_NCO <= '1';
                TX_temp <= '1';
                DE_temp <= '1';
                if stop_bits_reg = stop_bit then
                    next_state <= PostDE;
                    rst_delay <= '1';
                    rst_stop_bits <= '1';
                end if;

            when PostDE =>
                DE_temp <= '1';
                TX_temp <= '1';
                en_delay <= '1';
                en_NCO <= '0'; -- El NCO no es necesario en esta fase, podemos deshabilitarlo
                if delay_cnt_reg = to_unsigned(DELAY_CYCLES - 1, 14) then
                    next_state <= Idle;
                end if;

        end case;
    end process;

    -- State register
    process(Clk, Reset)
    begin
        if Reset = '0' then
            current_state <= Idle;
        elsif rising_edge(Clk) then
            current_state <= next_state;
        end if;
    end process;

    -- Outputs
    TX <= TX_temp;
    EOT <= EOT_temp;
    DE <= DE_temp;

end Behavioral;
