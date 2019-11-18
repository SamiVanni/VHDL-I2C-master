--Created by Sami Vänni 25/10/2018
--This design unit functions as SCCB master device -IP
-- SCCB = Serial Camera Control Bus
-- Communication protocol similar to I2C, designed by OmniVision for controlling their CMOS camera IC:s


library ieee;
use ieee.std_logic_1164.all;

entity SCCB_master is

--------------------------------------
--**	ENTITY GENERIC DECLARATIONS **--
--------------------------------------

-----------------------------------
--**	ENTITY PORT DECLARATIONS **--
-----------------------------------

	port (
--SCCB-bus signals that are connected to the slave device
		SCCB_E 			: 	out 		std_logic;
		SIO_C		 		: 	out 		std_logic;
		SIO_D				: 	inout 	std_logic;

--SCCB_master data and address signals
		ID_address 		: 	in 		std_logic_vector(7 downto 0);
		Sub_address 	:	in 		std_logic_vector(7 downto 0);
		Data_in			: 	in			std_logic_vector(7 downto 0);
		Data_out			:	out		std_logic_vector(7 downto 0);
		SCCB_Slave_Data : 	out		std_logic_vector(7 downto 0);
		
--SCCB_master control signals
		OP_code			: 	in			std_logic_vector(2 downto 0);
		sys_clk			:	in			std_logic;
		ReadyPort		: 	out		std_logic
		);
end entity;


------------------------
--**	ARCHITECTURE  **--
------------------------

architecture SCCB of SCCB_master is

use work.OV7670_Controller;
use work.edge_latch;
use work.delay_edge_latch;

type instructions is (SCCB_write, SCCB_read, Data_read, Data_write, ID_read, Sub_read, NOP);
type ConStates is (ST1, ST2, ST3, ST4, ST5, ST6, ST7);
type SCCB_Read_States is (RT1, RT2);

signal ControllerState	: ConStates := ST1;

-- These states are used to simplify the read data from slave instruction.
signal Read_State			: SCCB_Read_States := RT1;

constant start_transmission 	: integer := 28;
 -- By changing stop_transmission variabe, we can shorten the transmission lenght.
 -- This is needed when SCCB_master reads data from the slave device.
signal stop_transmission 		: integer := 0; 

constant half_cycle 		: 	integer := 500;						--Cycle width for the SIO_C -transmission clock signal
signal instruction 		: 	instructions := NOP;					--Internal control signal							

signal SCCB_clk			: 	std_logic := '0';
signal SCCB_clk_Event	:	std_logic;
signal SCCB_clk_DelayedEvent : std_logic;
signal SCCB_clk_E			: 	std_logic := '0';						--Start of transmission
signal SIO_C_Mask			:	std_logic := '1';
signal Data_Mask			:	std_logic := '0';
signal Transmission_Ended: std_logic;

signal Data_Vector			: std_logic_vector(26 downto 0) := (18=>'Z', 9=>'Z', 0=>'Z', others => 'Z');
signal Data_Vector_count	: integer range -1 to 28 := 28;
constant Start_Listening	: integer := 18;
constant Stop_Listening		: integer := 10;
signal Ready					: std_logic := '1';


begin

--Mask signals force the controller signal to '1'
--
SIO_C <= SCCB_clk OR SIO_C_Mask;
SCCB_E <= '0';
ReadyPort <= Ready;



------------------------------------------
--**	SIO_C CLOCK GENERATOR -PROCESS  **--
------------------------------------------
--This process generates 10 us clock period signal from the 50 MHz system clock signal
--Clock is activated by the SCCB_clk_E signal from the instruction decoder

SIOC_clock_generator : process (sys_clk) is
variable counter : integer range 1 to half_cycle := 1;
begin

--Basic counter
if rising_edge(sys_clk) then
	if (SCCB_clk_E = '1') AND (Transmission_Ended = '0') then
		
		if counter = half_cycle then
		counter := 1;
		SCCB_clk <= NOT SCCB_clk;
		else
		counter := counter + 1;
		end if;
	--Reset clock and counter to default value when transmission ends, so that the data line keeps on sync next time SCCB_write or read
	--instruction is ordered by the instruction decoder.
	elsif Transmission_Ended = '1' then 
	SCCB_clk <= '0';
	counter := 1;
	end if;
end if;

end process;
	
------------------------------
--** CONTROLLER -PROCESS  **--
------------------------------
controller : process (sys_clk) is

begin
if falling_edge(sys_clk) then
case ControllerState is
-----------------------------------------------------------
--These three states are responsible for masking SIO_C and SIO_D 
--during start of transmission.

--Transmission_Ended -signal is just a communication between instruction decoder
--and the Controller. It basically controls the flow of the SCCB_master resetting values,
--allowing other instructions to be carried out.
----------------------------------------------------------- START OF TRANSMISSION
--																				Data_Vector_count = 0
when ST1 =>
if SCCB_clk_E = '1' AND Data_Vector_count = start_transmission then
Transmission_Ended <= '0';	
Data_Mask <= '1';
ControllerState <= ST2;
end if;

when ST2 =>
if SCCB_clk_Event = '1' AND SCCB_clk = '1' then
Data_Mask <= '0';
ControllerState <= ST3;
end if;

when ST3 =>
if SCCB_clk_Event = '1' AND SCCB_clk = '0' then
SIO_C_Mask <= '0';
ControllerState <= ST4;
end if;
------------------------------------------------------------END
--																				Data_Vector_count 1
-- Phase 1, Phase 2 and depending on the instruction phase 3
-- is happening now and the Controller is waiting for the
-- stop of transmission
--
------------------------------------------------------------Stop of transmission
--																				Start
when ST4 =>
if Data_Vector_count = stop_transmission then
ControllerState <= ST5;
end if;

when ST5 =>
if SCCB_clk_Event = '1' AND SCCB_clk = '1' then
SIO_C_Mask <= '1';
ControllerState <= ST6;
end if;

when ST6 =>
if SCCB_clk_Event = '1' AND SCCB_clk = '0' then
Data_Mask <= '1';
ControllerState <= ST7;
end if;

when ST7 =>
if SCCB_clk_Event = '1' AND SCCB_clk = '1' then
Data_Mask <= '1';
ControllerState <= ST1;
Transmission_Ended <= '1';
end if;
------------------------------------------------------------End
end case;
end if;
end process;

---------------------------------
--**	DATA HANDLER -PROCESS  **--
---------------------------------
--This process handles the Data_Vector_count that determines what data
--to put on the bus

Data_Handler : process (sys_clk) is
variable Data_Bit : std_logic;
variable ST_Delay : std_logic := '0';
begin
if rising_edge(sys_clk) then

--Because SCCB-specification requires MSB to go on the data line first and we use big endian -data format, we have to start the
--Data vectors iteration counter from the max value and go towards the min value. When we reach the min value or the transmission
--ended, we reset the counter to the max value.

if SCCB_clk_E = '0' then
SIO_D <= '1';
else
		--This condition is because of timing requirements on the start of SCCB_read
	if Data_Vector_Count = Start_Transmission AND Instruction = SCCB_read then
		if SCCB_clk_DelayedEvent = '1' AND SCCB_clk = '0' then
			if ST_Delay = '0' then
			ST_Delay := '1';
			elsif ST_Delay = '1' then
			Data_Vector_Count <= Data_Vector_count -1;
			ST_Delay := '0';
			end if;
		end if;
	else
		if SCCB_clk_DelayedEvent = '1' AND SCCB_clk = '0' then
			if Data_Vector_count = stop_transmission -1 then
			Data_Vector_count <= start_transmission;
			elsif (Read_State = RT2) AND (Data_Vector_count = Stop_Listening -1) then
			Data_Vector_Count <= Stop_Transmission;
			else
			Data_Vector_count <= Data_Vector_count -1;
			end if;
		end if;
	end if;
	
	-- Outputs or inputs the SIO_D -port
	-- Data_Bit is used to hold the current data value and driven to the (Data_Bit OR Data_Mask) -gate.
	-- This is because multiple processes cant drive the same signals, but we can produce single signals out of combinatory logic.
	
		------------------------------- 2
		--This checks if DVC is in start or stop phase
		if (Data_Vector_count = start_transmission) OR (Data_Vector_count = stop_transmission) OR (Data_Vector_count = stop_transmission -1)  then
		Data_Bit := '0';
		SIO_D <= Data_Bit OR Data_Mask;
		else
			------------------------------------------------------------------ 3
			--When DVC is in the Start listening region, instead of writing Data Handler
			--starts reading the Data line. Exception is when DVC is on top on NA bit then Data Handler
			--drives the SIO_D high. This all triggers on the falling edge of SCCB_clk
			if (Read_State = RT2) AND (Data_Vector_count = Stop_Listening) then
				SIO_D <= '1';	
			
			elsif NOT((Read_State = RT2) AND (Data_Vector_count <= Start_Listening) AND (Data_Vector_count > Stop_Listening)) then
				Data_Bit := Data_Vector(Data_Vector_count - 1);
					-------------------------------------------------------------4
					--Checks if Count is on top of 9th bit = Dont Care bit
					if Data_Bit = 'Z' then
					SIO_D <= 'Z';
					else
					SIO_D <= Data_Bit OR Data_Mask;
					end if;
					-------------------------------------------------------------4
			end if;
			------------------------------------------------------------------ 3
		end if;
		-------------------------------- 2
end if;

--This if statement has to be separated from the data writer chain, because it triggers on the rising edge of SCCB_clk.
--Data needs to be read from the Data line on the rising edge..
Listener : if SCCB_clk_Event = '1' AND SCCB_clk = '1' then
	if (Read_State = RT2) AND (Data_Vector_count <= Start_Listening) AND (Data_Vector_count > Stop_Listening) then
		SCCB_Slave_Data(Data_Vector_count-11) <= SIO_D;
	end if;
end if;

if Transmission_Ended = '1' then
Data_Vector_count <= start_transmission;
end if;
end if;
end process;

----------------------------------------
--**	INSTRUCTION DECODER -PROCESS  **--
----------------------------------------
-- When SCCB master device has completed the previous instruction and transmission_ended = '1', then the next falling
-- edge of system clock triggers the process that reads next instruction from the OP_code -port. On the next rising edge of system
-- clock Data Vector is loaded with the data specified byt the OP_code -value loaded.

Instruction_decoder : process (sys_clk, Transmission_Ended)  is

variable RT2_Delay : std_logic := '0';

begin
if rising_edge(sys_clk) then




	if Ready = '1' then
		if 		OP_code = B"000" then
		instruction <= SCCB_write;
		Ready <= '0';
		elsif 	OP_code = B"001" then 
		instruction <= SCCB_read;
		Ready <= '0';
		
		elsif 	OP_code = B"010" then 
		instruction <= Data_read;
		Data_Vector(8 downto 1) <= Data_In;   --8 downto 1
		
		elsif 	OP_code = B"011" then 
		instruction <= Data_write;
		Data_out <= Data_Vector(8 downto 1);
		
		elsif 	OP_code = B"100" then 
		instruction <= ID_read;
		Data_Vector(26 downto 19) <= ID_address; --26 downto 19
		
		elsif 	OP_code = B"101" then 
		instruction <= Sub_read;
		Data_Vector(17 downto 10) <= Sub_address;
		
		else 		instruction <= NOP;
		end if;
	end if;


	if Ready <= '0' then
		if instruction = SCCB_write then
		
			Data_Vector(19) <= '0'; --Write Mode bit
			SCCB_clk_E <= '1';
			stop_transmission <= 0;
			if Transmission_Ended = '1' AND SCCB_clk_E = '1' then
			SCCB_clk_E <= '0';
			ready <= '1';
			end if;
		-- SCCB_read description
		-- In RT1 SCCB_master writes Slave ID address and Sub address to the bus
		-- In RT2 SCCB_master drives the SIO_C clock line and reads data according the specs
		-- When data is read End_Data_Read signal is driven high, Because at the end of RT1 Controller drives End_Transmission and
		-- so the Transmission_Ended signal activates.
			elsif instruction = SCCB_read then
				case Read_State is
				
				when RT1 =>
				RT2_Delay := '0';
				Data_Vector(19) <= '0'; --Write Mode bit
				SCCB_clk_E <= '1';
				stop_transmission <= 9;
				if Transmission_Ended = '1' AND SCCB_clk_E = '1' then
				Read_State <= RT2;
				SCCB_clk_E <= '0';
				end if;
				
				when RT2 =>
				if RT2_Delay = '1' then
					SCCB_clk_E <= '1';
					Data_Vector(19) <= '1'; --Read Mode bit on the ID address word
					if Transmission_Ended = '1' AND SCCB_clk_E = '1' then
					SCCB_clk_E <= '0';
					Read_State <= RT1;
					Ready <= '1';
					end if;
				else
				RT2_Delay := '1';
				end if;
				
				end case;
		end if;

	end if;
end if;


end process;
	
SCCB_clk_edgedetector : entity edge_latch
	port map (
		sys_clk 		=> sys_clk,
		D_in			=> SCCB_clk,
		D_out			=> SCCB_clk_Event
	);

SCCB_clk_edgedetector_Delayed : entity delay_edge_latch
	generic map (
		Delay 		=> (half_cycle/2)
	)
	port map (
		sys_clk 		=> sys_clk,
		D_in			=> SCCB_clk,
		D_out			=> SCCB_clk_DelayedEvent,
		reset			=> SCCB_clk_E
	);

end architecture;