
library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;
   
   
entity CONFIGURABLE_SERIAL is

  port (
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
    baud_sel  : in std_logic_vector(5 downto 0); -- configurable to 54 standard bps
    stop_bit  : in std_logic_vector(1 downto 0);  -- 1, 1.5 or 2 stop bits
    parity    : in std_logic_vector(2 downto 0);  -- 0→Even, 1→Odd, 2→Mark(=1), 3→Space(=0), 4→parity disabled
    bit_order : in std_logic;                     -- 0→LSB-first (default), 1→MSB-first
    data_bits  : in std_logic_vector(2 downto 0)); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b

end CONFIGURABLE_SERIAL;

architecture RTL of CONFIGURABLE_SERIAL is
 
 ------------------------------------------------------------------------
  -- Components for Transmitter Block
  ------------------------------------------------------------------------

  component TX_CONFIGURABLE_SERIAL
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
  end component;

  ------------------------------------------------------------------------
  -- Components for Receiver Block
  ------------------------------------------------------------------------

  component ShiftRegister
    port (
      Reset     : in  STD_LOGIC;
      Clk       : in  STD_LOGIC;
      Enable    : in  STD_LOGIC;                 -- pulso por bit recibido
      data_bits : in  std_logic_vector(2 downto 0); -- 0→5b, 1→6b, 2→7b, 3→8b, 4→9b   
      bit_order : in  std_logic;                 -- '0' = LSB-first, '1' = MSB-first
      D         : in  STD_LOGIC;                 -- bit muestreado (en el centro)
      Q         : out STD_LOGIC_VECTOR (8 downto 0));  -- palabra NORMALIZADA (LSB en Q(0))
  end component;

  component RX_CONFIGURABLE_SERIAL
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
  end component;

  component fifo_generator_0
    port (
      clk   : IN  std_logic;
      srst  : IN  std_logic;
      din   : IN  std_logic_VECTOR(8 downto 0);
      wr_en : IN  std_logic;
      rd_en : IN  std_logic;
      dout  : OUT std_logic_VECTOR(8 downto 0);
      full  : OUT std_logic;
      empty : OUT std_logic);
  end component;
  


  ------------------------------------------------------------------------
  -- Internal Signals
  ------------------------------------------------------------------------

  signal Data_FF    : std_logic_vector(8 downto 0);
  signal StartTX    : std_logic;  -- start signal for transmitter
  signal LineRD_in  : std_logic;  -- internal RX line
  signal Valid_out  : std_logic;  -- valid bit at the receiver
  signal Code_out   : std_logic;  -- bit at the receiver output
  signal sinit      : std_logic;  -- fifo reset
  signal Fifo_in    : std_logic_vector(8 downto 0);
  signal Fifo_write : std_logic;
  signal TX_RDY_i   : std_logic;
  
  -- data_read_flanc_detection:
  signal data_read_next, data_read_next_reg, data_read_flanc, data_read_reg_2 : std_logic;
  -- tx_send_flanc_detection:
  signal tx_send_next, tx_send_next_reg, tx_send_flanc, tx_send_reg_2 : std_logic;
  
  signal stop_bit_extended : std_logic_vector(2 downto 0);
  --signal stop_bit_shortened : std_logic_vector(1 downto 0);

begin  -- RTL
  stop_bit_extended <=  "011" when stop_bit = "10" else
                        "100" when stop_bit = "11" else
                        "010"; -- "00" or "01"
    --flanc detector double reg:
  process(Clk, Reset)
  begin
    if (Reset = '0') then 
        data_read_next_reg <= '0';
        data_read_reg_2 <= '0';
        tx_send_next_reg <= '0';
        tx_send_reg_2 <= '0';        
    elsif rising_edge(Clk) then
        data_read_next_reg <= data_read_next;
        data_read_reg_2 <= data_read_next_reg;
        tx_send_next_reg <= tx_send_next;
        tx_send_reg_2 <= tx_send_next_reg;        
    end if;
  end process;
  --next state logic
  data_read_next <= Data_read;
  tx_send_next <= TX_Send;  
  -- outpul logic
  data_read_flanc <= data_read_next_reg and not data_read_reg_2;
  tx_send_flanc <= tx_send_next_reg and not tx_send_reg_2;
  Transmitter: TX_CONFIGURABLE_SERIAL
    port map (
      Clk   => Clk,
      Reset => Reset,
      Start => StartTX,
      Data  => Data_FF,
      baud_sel  => baud_sel,
      stop_bit  => stop_bit_extended,
      parity    => parity,
      bit_order => bit_order,
      data_bits  => data_bits,      
      EOT   => TX_RDY_i,
      TX    => TD);

  Receiver: RX_CONFIGURABLE_SERIAL
    port map (
      Clk       => Clk,
      Reset     => Reset,
      baud_sel => baud_sel,
      stop_bit  => stop_bit_extended,
      parity    => parity,
      bit_order => bit_order,
      data_bits  => data_bits,                  
      LineRD_in => LineRD_in,
      shiftRegister_Q => Fifo_in,
      PAR_ERROR => PAR_ERROR,
      FRAME_ERROR => FRAME_ERROR,
      ERROR_OK => ERROR_OK,
      Valid_out => Valid_out,
      Code_out  => Code_out,
      Store_out => Fifo_write);

  Shift: ShiftRegister
    port map (
      Reset  => Reset,
      Clk    => Clk,
      Enable => Valid_Out,
      data_bits => data_bits,
      bit_order => bit_order,
      D      => Code_Out,
      Q      => Fifo_in);

  sinit <= not reset;
  
  Internal_memory: fifo_generator_0
    port map (
      clk   => Clk,
      srst  => sinit,
      din   => Fifo_in,
      wr_en => Fifo_write,
      rd_en => data_read_flanc,
      dout  => Data_out,
      full  => Full,
      empty => Empty);


  -- purpose: Clocking process for input protocol
  Clocking : process (Clk, Reset)
  begin
    if Reset = '0' then  -- asynchronous reset (active low)
      Data_FF   <= (others => '0');
      LineRD_in <= '1';
    elsif rising_edge(Clk) then  -- rising edge of the clock
      LineRD_in <= RD;
      if tx_send_flanc = '1' and TX_RDY_i = '1' then
        Data_FF <= Data_in;
        StartTX <= '1';
      else
        StartTX <= '0';
      end if;
    end if;
  end process Clocking;
  
 -- Logic : process (Valid_D, TX_RDY_i)
 -- begin
 --   Ack_in <= '1';
 --   if Valid_D = '0' and TX_RDY_i = '1' then
  --      Ack_in  <= '0';
   -- end if;
  --end process Logic;

  TX_RDY <= TX_RDY_i;

end RTL;

