
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_std.ALL;
USE IEEE.std_logic_unsigned.all;




entity TP4 is
Port 
(    i_A : in std_logic_vector  (3 downto 0);
     i_B : in std_logic_vector  (3 downto 0);
     CLK : in std_logic ;
     RST : in std_logic ;
     o_Q : out std_ulogic_vector (4 downto 0)
     );
end TP4;

architecture Behavioral of TP4 is
    
begin
process(clk,rst)
begin
    if (rst = '1')then
        o_Q <= "00000";
    elsif(clk'event AND clk ='1')then
        o_Q <= std_ulogic_vector (('0'&unsigned(i_A) +('0'&unsigned(i_B)))); 
    end if;

end process;

end Behavioral;
