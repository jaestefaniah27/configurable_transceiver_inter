
library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;
   
entity CONFIGURABLE_SERIAL_TOP is

  port (
    Reset : in std_logic;
    Clk   : in std_logic;
    -- PS ack_in, TX_RDY, Data_out, FULL, EMPTY, PAR ERROR, FRAME_ERROR
    PS_out : out std_logic_vector(13 downto 0);
    -- PS RX
    PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn : in std_logic_vector(26 downto 0);
    -- PS TX
    --PS_TX_DataIn_Send : in std_logic_vector(9 downto 0);
    -- PS CONFIG
    --PS_SERIAL_CONFIG : in std_logic_vector(15 downto 0);
    TX_RDY_EMPTY : out std_logic_vector(1 downto 0);
    TD : out std_logic;
    RD : in std_logic
    );

end CONFIGURABLE_SERIAL_TOP;

architecture RTL of CONFIGURABLE_SERIAL_TOP is
 
 ------------------------------------------------------------------------
  -- Components for Transmitter Block
  ------------------------------------------------------------------------

  component CONFIGURABLE_SERIAL
    Port (
    Reset     : in  std_logic;   -- Low-level asynchronous reset
    Clk       : in  std_logic;   -- System clock (100MHz), rising edge 
    -- TX	
    Data_in   : in  std_logic_vector(8 downto 0);  -- Parallel TX byte 
    TX_Send   : in  std_logic;   -- Send
    TX_RDY    : out std_logic;   -- System ready to transmit
    TD        : out std_logic;   -- Serial Transmission line
	-- RX
    RD        : in  std_logic;   -- Serial Reception line
    Data_out  : out std_logic_vector(8 downto 0);  -- Parallel RX byte
    Data_read : in  std_logic;   -- Send RX data to guest 
    Full      : out std_logic;   -- Internal RX memory full 
    Empty     : out std_logic;  -- Internal RX memory empty    
    PAR_ERROR : out std_logic;
    FRAME_ERROR : out std_logic;
    ERROR_OK  : in std_logic;
    -- CONFIG
    baud_sel  : in std_logic_vector(5 downto 0); -- configurable to 52 standard bps
    stop_bit  : in std_logic_vector(1 downto 0);  -- 1, 1.5 or 2 stop bits
    parity    : in std_logic_vector(2 downto 0);  -- 0→Even, 1→Odd, 2→Mark(=1), 3→Space(=0), 4→parity disabled
    bit_order : in std_logic;                     -- 0→LSB-first (default), 1→MSB-first
    data_bits  : in std_logic_vector(2 downto 0)); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b
  end component;
  
  signal TX_RDY_sig, FRAME_ERROR, PAR_ERROR, FULL, EMPTY_sig : std_logic;
  signal Data_out : std_logic_vector(8 downto 0);

begin  -- RTL
    TRANSCEIVER : CONFIGURABLE_SERIAL
        port map (
            Reset => Reset,
            Clk   => Clk,
            Data_in => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(8 downto 0),
            TX_Send => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(9),
            TX_RDY => TX_RDY_sig,
            TD => TD,
            RD => RD,
            Data_out => Data_out,
            Data_read => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(11),
            Full => FULL,
            Empty => EMPTY_sig,
            PAR_ERROR => PAR_ERROR,
            FRAME_ERROR => FRAME_ERROR,
            ERROR_OK =>  PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(10),
            baud_sel =>  PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(17 DOWNTO 12),
            stop_bit  => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(19 downto 18),
            parity    => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(22 downto 20),
            bit_order => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(26),
            data_bits => PS_SERIAL_CONFIG_DataRead_ErrorOk_Send_DataIn(25 downto 23)                 
        );

    PS_out <= TX_RDY_sig & FRAME_ERROR & PAR_ERROR & FULL & EMPTY_sig & Data_out;
    TX_RDY_EMPTY <= TX_RDY_sig & EMPTY_sig;
end RTL;

