function integer PortAddress;
  input integer i;
begin
  case (i)
    0 : PortAddress=`PORTA_ADDRESS;
    1 : PortAddress=`PORTB_ADDRESS;
    2 : PortAddress=`PORTC_ADDRESS;
    3 : PortAddress=`PORTD_ADDRESS;
    4 : PortAddress=`PORTE_ADDRESS;
    default: PortAddress=0;
  endcase
end
endfunction

function integer PortDDR;
  input integer i;
begin
  case (i)
    0 : PortDDR=`DDRA_ADDRESS;
    1 : PortDDR=`DDRB_ADDRESS;
    2 : PortDDR=`DDRC_ADDRESS;
    3 : PortDDR=`DDRD_ADDRESS;
    4 : PortDDR=`DDRE_ADDRESS;
    default: PortDDR=0;
  endcase
end
endfunction

function integer PortPin;
  input integer i;
begin
  case (i)
    0 : PortPin=`PINA_ADDRESS;
    1 : PortPin=`PINB_ADDRESS;
    2 : PortPin=`PINC_ADDRESS;
    3 : PortPin=`PIND_ADDRESS;
    4 : PortPin=`PINE_ADDRESS;
    5 : PortPin=`PINF_ADDRESS;
    default: PortPin=0;
  endcase
end
endfunction
