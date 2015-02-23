v 20130925 2
C 49400 53300 1 0 1 rs485-1.sym
{
T 47750 55100 5 10 0 0 0 6 1
device=MAX489
T 48050 53450 5 10 1 1 0 6 1
refdes=U2
T 47750 54900 5 10 0 0 0 6 1
footprint=SO8
}
C 61600 50400 1 0 0 ATmega164_TQFP.sym
{
T 65800 57900 5 10 1 1 0 6 1
refdes=U1
T 62000 58050 5 10 0 0 0 0 1
device=ATmega163_TQFP
T 62000 58250 5 10 0 0 0 0 1
footprint=TQFP44
}
T 42600 58900 9 10 1 0 0 6 1
12V
T 42600 58500 9 10 1 0 0 6 1
GND
T 42600 54400 9 10 1 0 0 6 1
RS485-A
T 42600 54000 9 10 1 0 0 6 1
RS485-B
N 47700 57900 47700 57700 4
N 47700 58800 47700 59000 4
N 50600 59000 67800 59000 4
N 62900 58100 62900 59000 4
N 63300 59000 63300 58100 4
N 63700 59000 63700 58100 4
N 64300 58100 64300 59000 4
C 47600 57400 1 0 0 gnd-1.sym
N 62400 50200 62400 50500 4
N 60500 50200 63600 50200 4
N 62800 50200 62800 50500 4
N 63200 50200 63200 50500 4
N 63600 49700 63600 50500 4
C 64200 49200 1 0 0 crystal-1.sym
{
T 64400 49700 5 10 0 0 0 0 1
device=CRYSTAL
T 64400 49500 5 10 1 1 0 0 1
refdes=Y1
T 64400 49900 5 10 0 0 0 0 1
symversion=0.1
T 64550 49000 5 10 1 1 0 3 1
value=20MHz
T 64200 49200 5 10 0 1 0 0 1
footprint=crystal-hc49-smt
}
N 64200 48800 64200 50500 4
N 64900 48800 64900 50500 4
C 64100 47600 1 0 0 gnd-1.sym
C 64700 48800 1 270 0 capacitor-1.sym
{
T 65400 48600 5 10 0 1 270 0 1
device=CAPACITOR
T 65000 48500 5 10 1 1 0 0 1
refdes=C4
T 65600 48600 5 10 0 0 270 0 1
symversion=0.1
T 65000 48000 5 10 1 1 0 0 1
value=22pF
T 65100 47600 5 10 0 1 0 0 1
footprint=0805
T 65000 47800 5 10 1 1 0 0 1
description=16V, NP0
}
C 64800 47600 1 0 0 gnd-1.sym
N 49400 54000 61700 54000 4
N 61700 53600 54300 53600 4
N 54300 53600 54300 55300 4
N 54300 54600 49400 54600 4
C 48500 51700 1 0 0 gnd-1.sym
N 48600 52000 48600 53300 4
N 48600 53000 49600 53000 4
N 49600 53000 49600 53700 4
N 49600 53700 49400 53700 4
C 54400 55300 1 90 0 resistor-1.sym
{
T 54000 55600 5 10 0 0 90 0 1
device=RESISTOR
T 54100 55500 5 10 1 1 90 0 1
refdes=R3
T 54600 55500 5 10 1 1 90 0 1
value=10k
T 54400 55300 5 10 0 0 90 0 1
footprint=0805
}
N 54300 56200 54300 59000 4
N 49400 54300 52100 54300 4
N 52100 52400 61700 52400 4
N 52100 54300 52100 52400 4
C 60600 50400 1 90 0 resistor-1.sym
{
T 60200 50700 5 10 0 0 90 0 1
device=RESISTOR
T 60300 50600 5 10 1 1 90 0 1
refdes=R4
T 60800 50600 5 10 1 1 90 0 1
value=10k
T 60600 50400 5 10 0 0 90 0 1
footprint=0805
}
C 63500 49400 1 0 0 gnd-1.sym
N 60500 50200 60500 50400 4
N 60500 51300 60500 52400 4
C 55600 56900 1 90 0 resistor-1.sym
{
T 55200 57200 5 10 0 0 90 0 1
device=RESISTOR
T 55300 57100 5 10 1 1 90 0 1
refdes=R2
T 55800 57100 5 10 1 1 90 0 1
value=330
T 55600 56900 5 10 0 0 90 0 1
footprint=0805
}
N 55500 57800 55500 59000 4
C 55700 55800 1 90 0 led-3.sym
{
T 55950 55750 5 10 1 1 90 0 1
device=AMBER LED
T 55150 56250 5 10 1 1 90 0 1
refdes=D2
T 55700 55800 5 10 0 0 0 0 1
footprint=1206
}
N 55500 56900 55500 56700 4
N 55500 54000 55500 55800 4
N 46100 56200 53200 56200 4
C 67600 58800 1 270 0 capacitor-1.sym
{
T 68300 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 67900 58500 5 10 1 1 0 0 1
refdes=C7
T 68500 58600 5 10 0 0 270 0 1
symversion=0.1
T 67900 58000 5 10 1 1 0 0 1
value=0.1uF
T 67600 58800 5 10 0 0 0 0 1
footprint=0805
T 67900 57800 5 10 1 1 0 0 1
description=16V
}
C 67700 57600 1 0 0 gnd-1.sym
N 67800 58800 67800 59000 4
N 61700 55400 60800 55400 4
C 60900 54100 1 0 0 gnd-1.sym
N 61000 54400 61000 55000 4
N 61000 55000 60800 55000 4
N 61700 54600 59200 54600 4
N 59200 54600 59200 55400 4
N 59200 55400 59400 55400 4
N 59400 55800 59200 55800 4
N 59200 55800 59200 56400 4
N 59200 56400 61300 56400 4
N 61300 56400 61300 55000 4
N 61300 55000 61700 55000 4
N 59400 55000 58900 55000 4
N 58900 55000 58900 49900 4
N 58900 49900 67300 49900 4
N 65500 49900 65500 50500 4
N 60800 55800 61000 55800 4
N 61000 55800 61000 59000 4
T 59600 56500 9 10 1 0 0 0 1
ICSP Header
T 60300 59100 9 10 1 0 0 3 1
Short J5 to power device from ICSP header
N 66100 57400 66500 57400 4
N 66500 57400 66500 59600 4
N 53100 59600 66500 59600 4
C 52600 60600 1 180 0 resistor-1.sym
{
T 52300 60200 5 10 0 0 180 0 1
device=RESISTOR
T 52200 60300 5 10 1 1 0 5 1
refdes=R7
T 52200 60700 5 10 1 1 0 3 1
value=20k
T 52600 60600 5 10 0 0 180 0 1
footprint=0805
}
N 53100 59600 53100 60500 4
N 46500 60500 51700 60500 4
C 65100 58300 1 0 0 capacitor-1.sym
{
T 65300 59000 5 10 0 1 0 0 1
device=CAPACITOR
T 65400 58800 5 10 1 1 180 0 1
refdes=C6
T 65300 59200 5 10 0 0 0 0 1
symversion=0.1
T 66200 58800 5 10 1 1 180 0 1
value=0.1uF
T 65100 58300 5 10 0 0 90 0 1
footprint=0805
T 65700 58300 5 10 1 1 0 0 1
description=16V
}
C 66300 58400 1 90 0 gnd-1.sym
N 65100 58500 64900 58500 4
N 64900 58500 64900 58100 4
C 55000 59900 1 0 0 gnd-1.sym
N 55100 60500 54500 60500 4
N 42200 51000 43000 51000 4
{
T 42100 51000 5 10 1 1 0 7 1
netname=VIN
}
N 43800 49400 43800 50400 4
N 44900 49400 44900 49700 4
N 44900 50600 44900 51000 4
C 44800 49100 1 0 0 gnd-1.sym
N 44600 51000 50000 51000 4
N 61700 53200 52900 53200 4
N 52900 49700 52900 53200 4
N 53300 52800 61700 52800 4
C 52500 43100 1 90 0 xbee-1.sym
{
T 48100 45500 5 10 0 0 90 0 1
device=XBEE
T 49100 44300 5 10 1 1 90 3 1
refdes=XU6
T 52500 43100 5 10 0 1 0 0 1
footprint=XBEE-SMT
T 52500 43100 5 10 0 0 0 0 1
device=2x NPPN101BFLD-RC
}
C 52200 42600 1 0 0 gnd-1.sym
N 52300 42900 52300 43100 4
N 47700 42700 49900 42700 4
N 49900 42700 49900 43100 4
N 50200 43100 50200 42500 4
N 48000 42500 50200 42500 4
N 49600 43100 49600 42900 4
N 44900 42900 49600 42900 4
N 46900 42900 46900 51000 4
N 48600 56200 48600 55200 4
C 46200 49700 1 90 0 resistor-1.sym
{
T 45800 50000 5 10 0 0 90 0 1
device=RESISTOR
T 45900 49900 5 10 1 1 90 0 1
refdes=R9
T 46400 49900 5 10 1 1 90 0 1
value=330
T 46200 49700 5 10 0 0 90 0 1
footprint=0805
}
N 46100 50600 46100 51000 4
C 46300 48600 1 90 0 led-3.sym
{
T 46550 48550 5 10 1 1 90 0 1
device=GREEN LED
T 45750 49050 5 10 1 1 90 0 1
refdes=D3
T 46300 48600 5 10 0 0 0 0 1
footprint=1206
}
N 46100 49700 46100 49500 4
C 46000 48100 1 0 0 gnd-1.sym
N 46100 48400 46100 48600 4
C 45900 42700 1 270 0 capacitor-1.sym
{
T 46600 42500 5 10 0 1 270 0 1
device=CAPACITOR
T 46200 42400 5 10 1 1 0 0 1
refdes=C8
T 46800 42500 5 10 0 0 270 0 1
symversion=0.1
T 46200 41900 5 10 1 1 0 0 1
value=1uF
T 45900 42700 5 10 0 0 0 0 1
footprint=0805
T 46200 41700 5 10 1 1 0 0 1
description=16V
}
C 44700 42700 1 270 0 capacitor-1.sym
{
T 45400 42500 5 10 0 1 270 0 1
device=CAPACITOR
T 45000 42400 5 10 1 1 0 0 1
refdes=C9
T 45600 42500 5 10 0 0 270 0 1
symversion=0.1
T 45000 41900 5 10 1 1 0 0 1
value=8.2pF
T 44700 42700 5 10 0 0 0 0 1
footprint=0805
T 45000 41700 5 10 1 1 0 0 1
description=16V, NP0
}
N 46100 42700 46100 42900 4
N 44900 42700 44900 42900 4
C 44800 41500 1 0 0 gnd-1.sym
C 46000 41500 1 0 0 gnd-1.sym
T 46800 43000 9 10 1 0 0 6 1
Place C11 & C12 near XBee pin 1
C 67400 50300 1 90 0 resistor-1.sym
{
T 67000 50600 5 10 0 0 90 0 1
device=RESISTOR
T 67100 50500 5 10 1 1 90 0 1
refdes=R10
T 67600 50500 5 10 1 1 90 0 1
value=10k
T 67400 50300 5 10 0 0 90 0 1
footprint=0805
}
C 67100 49500 1 270 0 capacitor-1.sym
{
T 67800 49300 5 10 0 1 270 0 1
device=CAPACITOR
T 67400 49200 5 10 1 1 0 0 1
refdes=C10
T 68000 49300 5 10 0 0 270 0 1
symversion=0.1
T 67400 48700 5 10 1 1 0 0 1
value=1uF
T 67100 49500 5 10 0 0 0 0 1
footprint=0805
T 67400 48500 5 10 1 1 0 0 1
description=16V
}
C 67200 48300 1 0 0 gnd-1.sym
N 67300 49500 67300 50300 4
N 67300 51200 67300 59000 4
T 68200 49800 9 10 1 0 90 3 1
Optional (improved noise immunity)
T 67000 40900 9 10 1 0 0 0 1
MRBus <-> MRBee (XBee) Access Point
T 66800 40600 9 10 1 0 0 0 1
mrb-ap.sch
T 67000 40300 9 10 1 0 0 0 1
1
T 68500 40300 9 10 1 0 0 0 1
1
T 70800 40300 9 10 1 0 0 0 1
Michael Petersen
C 40000 40000 0 0 0 title-bordered-D.sym
N 53700 52000 61700 52000 4
N 54100 51600 61700 51600 4
T 58400 43500 9 10 1 0 0 2 3
Notes:
1) All capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
N 53200 56200 53200 59000 4
C 57600 57900 1 90 0 resistor-1.sym
{
T 57200 58200 5 10 0 0 90 0 1
device=RESISTOR
T 57300 58100 5 10 1 1 90 0 1
refdes=R1
T 57800 58100 5 10 1 1 90 0 1
value=330
T 57600 57900 5 10 0 0 90 0 1
footprint=0805
}
N 57500 58800 57500 59000 4
C 57700 56800 1 90 0 led-3.sym
{
T 57950 56750 5 10 1 1 90 0 1
device=GREEN LED
T 57150 57250 5 10 1 1 90 0 1
refdes=D1
T 57700 56800 5 10 0 0 0 0 1
footprint=1206
}
N 57500 57900 57500 57700 4
C 57400 56300 1 0 0 gnd-1.sym
N 57500 56600 57500 56800 4
C 43700 45700 1 0 0 led-3.sym
{
T 43450 45450 5 10 1 1 0 0 1
device=RED (ASSOC) LED
T 44150 46250 5 10 1 1 0 0 1
refdes=D5
T 43700 45700 5 10 0 0 270 0 1
footprint=1206
}
N 45400 45900 44600 45900 4
C 49900 41700 1 0 0 resistor-1.sym
{
T 50200 42100 5 10 0 0 0 0 1
device=RESISTOR
T 50100 42000 5 10 1 1 0 0 1
refdes=R12
T 50100 41500 5 10 1 1 0 0 1
value=330
T 49900 41700 5 10 0 0 0 0 1
footprint=0805
}
C 48800 41600 1 0 0 led-3.sym
{
T 49350 41450 5 10 1 1 0 5 1
device=YELLOW (RSSI) LED
T 49250 42150 5 10 1 1 0 0 1
refdes=D6
T 48800 41600 5 10 0 0 270 0 1
footprint=1206
}
N 49900 41800 49700 41800 4
N 46300 45900 51100 45900 4
N 51100 45900 51100 45500 4
N 50800 41800 51100 41800 4
N 51100 41800 51100 43100 4
C 48100 41300 1 0 0 gnd-1.sym
N 48200 41800 48800 41800 4
N 48200 41800 48200 41600 4
N 43100 45900 43700 45900 4
N 43100 45900 43100 45700 4
C 43000 45400 1 0 0 gnd-1.sym
N 53300 52800 53300 49300 4
N 53700 48900 53700 52000 4
N 54100 48500 54100 51600 4
N 52000 45500 52000 46500 4
C 59400 54800 1 0 0 header3x2-1.sym
{
T 59400 56400 5 10 0 1 0 0 1
device=HEADER6
T 60000 56100 5 10 1 1 0 0 1
refdes=J3
T 59400 54800 5 10 0 0 0 0 1
footprint=JUMPER3x2
}
C 43500 53900 1 0 1 termblk2-1.sym
{
T 42500 54550 5 10 0 0 0 6 1
device=TERMBLK2
T 43100 54800 5 10 1 1 0 6 1
refdes=J2
T 43500 53900 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 43500 58400 1 0 1 termblk2-1.sym
{
T 42500 59050 5 10 0 0 0 6 1
device=TERMBLK2
T 43100 59300 5 10 1 1 0 6 1
refdes=J1
T 43500 58400 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
N 55100 60500 55100 60200 4
N 52600 60500 53600 60500 4
C 64300 46000 1 0 0 hole-1.sym
{
T 64300 46000 5 10 0 1 0 0 1
device=HOLE
T 64500 46600 5 10 1 1 0 4 1
refdes=H1
T 64300 46000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 64800 46000 1 0 0 hole-1.sym
{
T 64800 46000 5 10 0 1 0 0 1
device=HOLE
T 65000 46600 5 10 1 1 0 4 1
refdes=H2
T 64800 46000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 65300 46000 1 0 0 hole-1.sym
{
T 65300 46000 5 10 0 1 0 0 1
device=HOLE
T 65500 46600 5 10 1 1 0 4 1
refdes=H3
T 65300 46000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 65800 46000 1 0 0 hole-1.sym
{
T 65800 46000 5 10 0 1 0 0 1
device=HOLE
T 66000 46600 5 10 1 1 0 4 1
refdes=H4
T 65800 46000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 64400 48800 1 90 1 capacitor-1.sym
{
T 63700 48600 5 10 0 1 270 2 1
device=CAPACITOR
T 64100 48500 5 10 1 1 0 6 1
refdes=C3
T 63500 48600 5 10 0 0 270 2 1
symversion=0.1
T 64100 48000 5 10 1 1 0 6 1
value=22pF
T 64000 47600 5 10 0 1 0 6 1
footprint=0805
T 64100 47800 5 10 1 1 0 6 1
description=16V, NP0
}
C 47500 58800 1 270 0 capacitor-1.sym
{
T 48200 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 47800 58500 5 10 1 1 0 0 1
refdes=C1
T 48400 58600 5 10 0 0 270 0 1
symversion=0.1
T 47800 58000 5 10 1 1 0 0 1
value=68uF
T 47800 57800 5 10 1 1 0 0 1
description=25V, Electrolytic
T 47500 58800 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
N 45200 59000 43500 59000 4
C 45200 58800 1 0 0 schottky-1.sym
{
T 45522 59472 5 10 0 0 0 0 1
device=DIODE
T 45500 59300 5 10 1 1 0 0 1
refdes=D4
T 45541 59632 5 10 0 1 0 0 1
footprint=SOD123
T 45000 58500 5 10 1 1 0 0 1
model=MBR0520LT1G
}
N 46500 60500 46500 59000 4
N 46100 59000 49000 59000 4
{
T 47000 59000 5 10 1 1 0 0 1
netname=VIN
}
C 43600 58100 1 0 0 gnd-1.sym
N 43500 58600 43700 58600 4
N 43700 58600 43700 58400 4
C 49700 57400 1 0 0 gnd-1.sym
N 49800 58400 49800 57700 4
C 51300 58800 1 270 0 capacitor-1.sym
{
T 52000 58600 5 10 0 1 270 0 1
device=CAPACITOR
T 51600 58500 5 10 1 1 0 0 1
refdes=C2
T 52200 58600 5 10 0 0 270 0 1
symversion=0.1
T 51600 58000 5 10 1 1 0 0 1
value=1uF
T 51300 58800 5 10 0 0 0 0 1
footprint=0805
T 51600 57800 5 10 1 1 0 0 1
description=16V
}
C 51400 57400 1 0 0 gnd-1.sym
N 51500 57700 51500 57900 4
N 51500 58800 51500 59000 4
C 49000 58400 1 0 0 78l05-1.sym
{
T 50600 59700 5 10 0 0 0 0 1
device=7805
T 50400 59400 5 10 1 1 0 6 1
refdes=U3
T 49000 58400 5 10 1 1 0 0 1
footprint=SOT89
}
C 44700 50600 1 270 0 capacitor-1.sym
{
T 45400 50400 5 10 0 1 270 0 1
device=CAPACITOR
T 45000 50300 5 10 1 1 0 0 1
refdes=C15
T 45600 50400 5 10 0 0 270 0 1
symversion=0.1
T 45000 49800 5 10 1 1 0 0 1
value=1uF
T 44700 50600 5 10 0 0 0 0 1
footprint=0805
T 45000 49600 5 10 1 1 0 0 1
description=16V
}
C 42300 50600 1 270 0 capacitor-1.sym
{
T 43000 50400 5 10 0 1 270 0 1
device=CAPACITOR
T 42600 50300 5 10 1 1 0 0 1
refdes=C14
T 43200 50400 5 10 0 0 270 0 1
symversion=0.1
T 42600 49800 5 10 1 1 0 0 1
value=10uF
T 42300 50600 5 10 0 0 0 0 1
footprint=0805
T 42600 49600 5 10 1 1 0 0 1
description=25V
}
N 42500 49400 42500 49700 4
C 42400 49100 1 0 0 gnd-1.sym
C 43700 49100 1 0 0 gnd-1.sym
C 49600 56000 1 270 0 capacitor-1.sym
{
T 50300 55800 5 10 0 1 270 0 1
device=CAPACITOR
T 49900 55700 5 10 1 1 0 0 1
refdes=C5
T 50500 55800 5 10 0 0 270 0 1
symversion=0.1
T 49900 55200 5 10 1 1 0 0 1
value=0.1uF
T 49600 56000 5 10 0 0 0 0 1
footprint=0805
T 49900 55000 5 10 1 1 0 0 1
description=16V
}
C 49700 54800 1 0 0 gnd-1.sym
N 49800 56000 49800 56200 4
C 43000 50400 1 0 0 lm7805-1.sym
{
T 44400 51400 5 10 1 1 0 6 1
refdes=U4
T 43000 50400 5 10 0 1 0 0 1
footprint=RECOM-TO220
T 43300 51600 5 10 1 1 0 0 1
device=R-78E3.3-0.5
}
C 46300 53500 1 0 0 header4-1.sym
{
T 47300 54150 5 10 0 0 0 0 1
device=HEADER3
T 47100 54300 5 10 1 1 270 3 1
refdes=JP2_3
T 46300 53500 5 10 0 0 0 6 1
footprint=JUMPER4-SMT
}
N 44800 54500 45000 54700 4
N 45900 53900 46100 54100 4
N 45000 53900 44800 54100 4
N 44800 54100 43500 54100 4
N 46100 54100 46300 54100 4
N 44800 54500 43500 54500 4
N 46100 54500 45900 54700 4
N 46100 54500 46300 54500 4
N 44600 54500 44600 55400 4
N 44600 55400 47400 55400 4
N 47400 55400 47400 54700 4
N 47400 54700 47800 54700 4
N 47800 53900 47400 53900 4
N 47400 53900 47400 53200 4
N 47400 53200 44600 53200 4
N 44600 53200 44600 54100 4
N 46300 54900 46100 54900 4
N 46100 54900 46100 56200 4
N 46300 53700 46100 53700 4
N 46100 52700 46100 53700 4
N 46100 52700 48600 52700 4
N 44100 54500 44100 56600 4
N 43900 54100 43900 56400 4
N 42500 51000 42500 50600 4
C 42200 55600 1 0 0 rj45-1.sym
{
T 42200 58500 5 10 0 0 0 0 1
device=RJ45
T 42500 55500 5 10 1 1 0 5 1
footprint=modular_8p8c_lp
T 42200 57500 5 10 1 1 0 0 1
refdes=J4
}
N 44500 57200 43100 57200 4
N 43800 56800 43100 56800 4
N 43100 57000 43500 57000 4
N 43500 55500 43500 57000 4
N 43500 56200 43100 56200 4
C 43400 55200 1 0 0 gnd-1.sym
N 43100 56400 43900 56400 4
N 43100 56600 44100 56600 4
N 43800 56800 43800 57200 4
C 54500 60600 1 180 0 resistor-1.sym
{
T 54200 60200 5 10 0 0 180 0 1
device=RESISTOR
T 54100 60300 5 10 1 1 0 5 1
refdes=R8
T 54100 60700 5 10 1 1 0 3 1
value=10k
T 54500 60600 5 10 0 0 180 0 1
footprint=0805
}
C 46300 46000 1 180 0 resistor-1.sym
{
T 46000 45600 5 10 0 0 180 0 1
device=RESISTOR
T 45900 46100 5 10 1 1 0 3 1
refdes=R11
T 45900 45700 5 10 1 1 0 5 1
value=330
T 46300 46000 5 10 0 0 180 0 1
footprint=0805
}
C 45900 54800 1 180 0 resistor-1.sym
{
T 45600 54400 5 10 0 0 180 0 1
device=RESISTOR
T 45500 54900 5 10 1 1 0 3 1
refdes=R5
T 45500 54500 5 10 1 1 0 5 1
value=2k
T 45900 54800 5 10 0 0 180 0 1
footprint=0805
}
C 45900 54000 1 180 0 resistor-1.sym
{
T 45600 53600 5 10 0 0 180 0 1
device=RESISTOR
T 45500 54100 5 10 1 1 0 3 1
refdes=R6
T 45500 53700 5 10 1 1 0 5 1
value=2k
T 45900 54000 5 10 0 0 180 0 1
footprint=0805
}
C 49300 47700 1 0 0 txb0104_QFN.sym
{
T 51300 50200 5 10 1 1 0 0 1
refdes=U5
T 50400 50100 5 10 1 1 0 4 1
device=TXB0104
T 49600 54700 5 10 0 0 0 0 1
footprint=TXB0104
}
N 50000 50800 50000 51000 4
N 50800 50800 50800 56200 4
N 50800 47700 50800 47000 4
N 50800 47000 49000 47000 4
N 49000 47000 49000 51000 4
C 49900 47200 1 0 0 gnd-1.sym
N 50000 47500 50000 47700 4
N 52900 49700 51500 49700 4
N 51500 49300 53300 49300 4
N 51500 48900 53700 48900 4
N 51500 48500 54100 48500 4
N 47700 42700 47700 49700 4
N 47700 49700 49300 49700 4
N 49300 49300 48000 49300 4
N 48000 49300 48000 42500 4
N 48300 46500 52000 46500 4
N 48300 46500 48300 48900 4
N 48300 48900 49300 48900 4
N 50800 45500 50800 46200 4
N 50800 46200 48600 46200 4
N 48600 46200 48600 48500 4
N 48600 48500 49300 48500 4
N 50400 47700 50400 47600 4
N 50400 47600 50000 47600 4
N 44500 57200 44500 59000 4
