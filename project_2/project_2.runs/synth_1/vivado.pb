
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
create_project: 2

00:00:052

00:00:062	
467.8522	
185.598Z17-268h px� 
�
Command: %s
1870*	planAhead2�
�read_checkpoint -auto_incremental -incremental {C:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/utils_1/imports/synth_1/OTTER_MCU.dcp}Z12-2866h px� 
�
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2s
qC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/utils_1/imports/synth_1/OTTER_MCU.dcpZ12-5825h px� 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px� 
d
Command: %s
53*	vivadotcl23
1synth_design -top OTTER_MCU -part xc7a35tcpg236-1Z4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
z
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2	
xc7a35tZ17-347h px� 
j
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2	
xc7a35tZ17-349h px� 
D
Loading part %s157*device2
xc7a35tcpg236-1Z21-403h px� 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
o
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
2Z8-7079h px� 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px� 
N
#Helper process launched with PID %s4824*oasys2
35948Z8-7075h px� 
�
%s*synth2{
yStarting RTL Elaboration : Time (s): cpu = 00:00:05 ; elapsed = 00:00:06 . Memory (MB): peak = 1304.125 ; gain = 441.879
h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2

reg_adr12
wire2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
2928@Z8-11241h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2

reg_adr22
wire2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
2938@Z8-11241h px� 
�
5undeclared symbol '%s', assumed default net type '%s'7502*oasys2

imgen_ir2
wire2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
3018@Z8-11241h px� 
�
synthesizing module '%s'%s4497*oasys2
	OTTER_MCU2
 2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
818@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
Hazard_Module2
 2t
pC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/Branch_Hazard_Module.v2
238@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Hazard_Module2
 2
02
12t
pC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/Branch_Hazard_Module.v2
238@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
HazardDetectorAndStaller2
 2y
uC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/HazardDetectorAndStaller.sv2
238@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
HazardDetectorAndStaller2
 2
02
12y
uC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/HazardDetectorAndStaller.sv2
238@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ForwardingUnit2
 2o
kC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/ForwardingUnit.sv2
238@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ForwardingUnit2
 2
02
12o
kC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.srcs/sources_1/new/ForwardingUnit.sv2
238@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
PC2
 2H
DC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC.sv2
98@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
PC_MUX2
 2L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC_MUX.sv2
98@Z8-6157h px� 
�
default block is never used226*oasys2L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC_MUX.sv2
228@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
PC_MUX2
 2
02
12L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC_MUX.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
PC_REG2
 2L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC_REG.sv2
98@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
PC_REG2
 2
02
12L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC_REG.sv2
98@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
PC2
 2
02
12H
DC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/PC.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

REG_FILE2
 2N
JC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/REG_FILE.sv2
108@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

REG_FILE2
 2
02
12N
JC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/REG_FILE.sv2
108@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ImmediateGenerator2
 2X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/ImmediateGenerator.sv2
98@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ImmediateGenerator2
 2
02
12X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/ImmediateGenerator.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
CU_DCDR2
 2M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/CU_DCDR.sv2
98@Z8-6157h px� 
�
default block is never used226*oasys2M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/CU_DCDR.sv2
1448@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
CU_DCDR2
 2
02
12M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/CU_DCDR.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
TwoMux2
 2L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/TwoMux.sv2
98@Z8-6157h px� 
�
default block is never used226*oasys2L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/TwoMux.sv2
188@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
TwoMux2
 2
02
12L
HC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/TwoMux.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
FourMux2
 2M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/FourMux.sv2
98@Z8-6157h px� 
�
default block is never used226*oasys2M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/FourMux.sv2
218@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
FourMux2
 2
02
12M
IC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/FourMux.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ALU2
 2I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/ALU.sv2
98@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ALU2
 2
02
12I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/ALU.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
BAG2
 2I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/BAG.sv2
98@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BAG2
 2
02
12I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/BAG.sv2
98@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
BCG2
 2I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/BCG.sv2
88@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BCG2
 2
02
12I
EC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/BCG.sv2
88@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Memory2
 2X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/otter_memory_v1_07.sv2
488@Z8-6157h px� 
�
,$readmem data file '%s' is read successfully3426*oasys2
Test_All.mem2X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/otter_memory_v1_07.sv2
738@Z8-3876h px� 
�
-case statement is not full and has no default155*oasys2X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/otter_memory_v1_07.sv2
958@Z8-155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Memory2
 2
02
12X
TC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/otter_memory_v1_07.sv2
488@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	OTTER_MCU2
 2
02
12E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
818@Z8-6155h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[rs1]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[rs2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[Utype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[Itype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[Stype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[Btype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[Jtype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[alu_src_a]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[alu_src_b]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[wb_sel]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[alu_fun]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[mem_we2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[mem_rden2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[branchStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[jalStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[jalrStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[alu_result]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[dout2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[func3]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
fetch2dec_reg[reg_wr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1288@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
dec2exec_reg[alu_result]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1268@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
dec2exec_reg[dout2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1268@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
dec2exec_reg[func3]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1268@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[pc_out]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[rs1]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[rs2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[Utype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[Itype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[Stype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[Btype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[Jtype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[alu_src_a]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[alu_src_b]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[alu_fun]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[mem_we2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[mem_rden2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[branchStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[jalStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[jalrStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[dout2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
exec2mem_reg[func3]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
4698@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[pc_out]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[rs1]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[rs2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[Utype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[Itype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[Stype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[Btype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[Jtype]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[alu_src_a]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[alu_src_b]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[alu_fun]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[mem_we2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[mem_rden2]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[branchStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[jalStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[jalrStr]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2
mem2wb_reg[func3]2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
1438@Z8-6014h px� 
�
0Net %s in module/entity %s does not have driver.3422*oasys2
	IOBUS_OUT2
	OTTER_MCU2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
868@Z8-3848h px� 
�
0Net %s in module/entity %s does not have driver.3422*oasys2

IOBUS_ADDR2
	OTTER_MCU2E
AC:/Users/Advika Deodhar/Desktop/otter_mcu_pipeline_template_v2.sv2
878@Z8-3848h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[31]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[30]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[29]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[28]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[27]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[26]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[25]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[24]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[23]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[22]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[21]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[20]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[19]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[18]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[17]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[16]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[15]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[14]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[13]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[12]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[11]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[10]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[9]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[8]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[7]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[6]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[5]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[4]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[3]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[2]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[1]2
BCGZ8-7129h px� 
k
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[0]2
BCGZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[31]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[30]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[29]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[28]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[27]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[26]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[25]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[24]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[23]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[22]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[21]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[20]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[19]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[18]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[17]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[16]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[15]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[14]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[13]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[12]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[11]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[10]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[9]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[8]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[7]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[6]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[5]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[4]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[3]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[2]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[1]2
	OTTER_MCUZ8-7129h px� 
x
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_OUT[0]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[31]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[30]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[29]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[28]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[27]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[26]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[25]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[24]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[23]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[22]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[21]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[20]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[19]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[18]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[17]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[16]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[15]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[14]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[13]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[12]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[11]2
	OTTER_MCUZ8-7129h px� 
z
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[10]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[9]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[8]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[7]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[6]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[5]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[4]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[3]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[2]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[1]2
	OTTER_MCUZ8-7129h px� 
y
9Port %s in module %s is either unconnected or has no load4866*oasys2
IOBUS_ADDR[0]2
	OTTER_MCUZ8-7129h px� 
p
9Port %s in module %s is either unconnected or has no load4866*oasys2
INTR2
	OTTER_MCUZ8-7129h px� 
�
%s*synth2{
yFinished RTL Elaboration : Time (s): cpu = 00:00:07 ; elapsed = 00:00:08 . Memory (MB): peak = 1428.531 ; gain = 566.285
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:07 ; elapsed = 00:00:08 . Memory (MB): peak = 1428.531 ; gain = 566.285
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:07 ; elapsed = 00:00:08 . Memory (MB): peak = 1428.531 ; gain = 566.285
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0292

1428.5312
0.000Z17-268h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
>

Processing XDC Constraints
244*projectZ1-262h px� 
=
Initializing timing engine
348*projectZ1-569h px� 
�
Parsing XDC File [%s]
179*designutils2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc8Z20-179h px� 
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
128@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
128@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
138@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
138@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
148@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
148@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
158@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
158@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
168@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
168@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
178@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
178@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
188@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
188@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
198@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
198@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
208@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
208@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
218@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
218@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
228@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
228@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
238@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
238@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
248@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
248@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
258@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
258@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
268@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
268@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
278@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
278@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[8]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
288@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
288@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[8]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
298@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
298@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[9]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
308@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
308@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[9]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
318@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
318@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[10]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
328@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
328@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[10]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
338@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
338@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[11]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
348@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
348@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[11]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
358@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
358@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[12]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
368@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
368@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[12]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
378@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
378@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[13]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
388@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
388@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[13]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
398@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
398@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[14]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
408@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
408@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[14]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
418@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
418@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[15]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
428@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
428@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
SWITCHES[15]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
438@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
438@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
478@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
478@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
488@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
488@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
498@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
498@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
508@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
508@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
518@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
518@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
528@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
528@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
538@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
538@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
548@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
548@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
558@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
558@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
568@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
568@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
578@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
578@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
588@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
588@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
598@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
598@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
608@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
608@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
618@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
618@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
628@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
628@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[8]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
638@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
638@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[8]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
648@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
648@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[9]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
658@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
658@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2	
LEDS[9]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
668@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
668@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[10]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
678@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
678@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[10]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
688@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
688@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[11]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
698@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
698@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[11]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
708@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
708@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[12]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
718@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
718@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[12]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
728@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
728@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[13]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
738@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
738@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[13]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
748@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
748@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[14]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
758@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
758@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[14]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
768@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
768@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[15]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
778@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
778@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2

LEDS[15]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
788@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
788@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
828@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
828@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[6]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
838@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
838@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
848@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
848@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[5]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
858@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
858@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
868@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
868@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[4]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
878@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
878@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
888@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
888@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
898@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
898@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
908@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
908@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
918@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
918@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
928@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
928@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
938@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
938@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
948@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
948@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
958@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
958@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
978@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
978@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
CATHODES[7]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
988@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
988@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1008@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1008@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[0]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1018@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1018@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1028@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1028@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[1]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1038@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1038@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1048@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1048@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[2]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1058@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1058@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1068@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1068@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
	ANODES[3]2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1078@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1078@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
BTNC2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1118@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1118@Z17-55h px�
�
No ports matched '%s'.
584*	planAhead2
BTNC2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1128@Z12-584h px�
�
"'%s' expects at least one object.
55*common2
set_property2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
1128@Z17-55h px�
�
Finished Parsing XDC File [%s]
178*designutils2T
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc8Z20-178h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2R
PC:/Users/Advika Deodhar/Desktop/OTTER_Diego/OTTER_Diego_Curiel/Basys3_Master.xdc2
.Xil/OTTER_MCU_propImpl.xdcZ1-236h px� 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0012

1479.0162
0.000Z17-268h px� 
l
!Unisim Transformation Summary:
%s111*project2'
%No Unisim elements were transformed.
Z1-111h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
 Constraint Validation Runtime : 2

00:00:002
00:00:00.0072

1479.0162
0.000Z17-268h px� 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
Finished Constraint Validation : Time (s): cpu = 00:00:16 ; elapsed = 00:00:17 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
D
%s
*synth2,
*Start Loading Part and Timing Information
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Loading part: xc7a35tcpg236-1
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:16 ; elapsed = 00:00:17 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
%s
*synth20
.Start Applying 'set_property' XDC Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:16 ; elapsed = 00:00:17 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322
"Memory:/memory_reg"Z8-7030h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:16 ; elapsed = 00:00:17 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Detailed RTL Component Info : 
h p
x
� 
(
%s
*synth2
+---Adders : 
h p
x
� 
F
%s
*synth2.
,	   2 Input   32 Bit       Adders := 5     
h p
x
� 
F
%s
*synth2.
,	   3 Input   32 Bit       Adders := 1     
h p
x
� 
&
%s
*synth2
+---XORs : 
h p
x
� 
H
%s
*synth20
.	   2 Input     32 Bit         XORs := 1     
h p
x
� 
+
%s
*synth2
+---Registers : 
h p
x
� 
H
%s
*synth20
.	               32 Bit    Registers := 24    
h p
x
� 
H
%s
*synth20
.	                4 Bit    Registers := 1     
h p
x
� 
H
%s
*synth20
.	                2 Bit    Registers := 4     
h p
x
� 
H
%s
*synth20
.	                1 Bit    Registers := 9     
h p
x
� 
&
%s
*synth2
+---RAMs : 
h p
x
� 
Z
%s
*synth2B
@	             512K Bit	(16384 X 32 bit)          RAMs := 1     
h p
x
� 
W
%s
*synth2?
=	             1024 Bit	(32 X 32 bit)          RAMs := 1     
h p
x
� 
'
%s
*synth2
+---Muxes : 
h p
x
� 
F
%s
*synth2.
,	   2 Input   32 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	   4 Input   32 Bit        Muxes := 4     
h p
x
� 
F
%s
*synth2.
,	   9 Input   32 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	  16 Input   32 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	   9 Input    4 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	  10 Input    4 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	   2 Input    2 Bit        Muxes := 8     
h p
x
� 
F
%s
*synth2.
,	   4 Input    2 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	  10 Input    2 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	  10 Input    1 Bit        Muxes := 7     
h p
x
� 
F
%s
*synth2.
,	   2 Input    1 Bit        Muxes := 9     
h p
x
� 
F
%s
*synth2.
,	   9 Input    1 Bit        Muxes := 1     
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Finished RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
p
%s
*synth2X
VPart Resources:
DSPs: 90 (col length:60)
BRAMs: 100 (col length: RAMB18 60 RAMB36 30)
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
E
%s
*synth2-
+Start Cross Boundary and Area Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px� 
�
?The signal %s was recognized as a true dual port RAM template.
3473*oasys2$
""OTTER_MCU/OTTER_REG_FILE/ram_reg"Z8-3971h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[31]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[30]2
BCGZ8-7129h px� 
l
9Port %s in module %s is either unconnected or has no load4866*oasys2
ir[29]2
BCGZ8-7129h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-71292
100Z17-14h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
LImplemented Non-Cascaded %s Ram (cascade_height = %s) of width %s for RAM %s4766*oasys2
Block2
12
322%
#"OTTER_MCU/OTTER_MEMORY/memory_reg"Z8-7030h px� 
�
?The signal %s was recognized as a true dual port RAM template.
3473*oasys2$
""OTTER_MCU/OTTER_REG_FILE/ram_reg"Z8-3971h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
h px� 
l
%s*synth2T
R---------------------------------------------------------------------------------
h px� 
R
%s*synth2:
8
Block RAM: Preliminary Mapping Report (see note below)
h px� 
�
%s*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
h px� 
�
%s*synth2�
�|Module Name | RTL Object              | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
h px� 
�
%s*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
h px� 
�
%s*synth2�
�|OTTER_MCU   | OTTER_MEMORY/memory_reg | 16 K x 32(READ_FIRST)  | W | R | 16 K x 32(WRITE_FIRST) |   | R | Port A and B     | 0      | 16     | 
h px� 
�
%s*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

h px� 
�
%s*synth2�
�Note: The table above is a preliminary report that shows the Block RAMs at the current stage of the synthesis flow. Some Block RAMs may be reimplemented as non Block RAM primitives later in the synthesis flow. Multiple instantiated Block RAMs are reported only once. 
h px� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
h px� 
l
%s*synth2T
R---------------------------------------------------------------------------------
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
@
%s
*synth2(
&Start Applying XDC Timing Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:34 ; elapsed = 00:00:36 . Memory (MB): peak = 1479.016 ; gain = 616.770
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
4
%s
*synth2
Start Timing Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
�The signal %s is implemented as distributed LUT RAM for the following reason(s): The timing constraints suggest that the chosen mapping will yield better timing results.
4036*oasys2$
""OTTER_MCU/OTTER_REG_FILE/ram_reg"Z8-5584h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2
}Finished Timing Optimization : Time (s): cpu = 00:00:50 ; elapsed = 00:00:51 . Memory (MB): peak = 1548.207 ; gain = 685.961
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s
*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!
Block RAM: Final Mapping Report
h p
x
� 
�
%s
*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
h p
x
� 
�
%s
*synth2�
�|Module Name | RTL Object              | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
h p
x
� 
�
%s
*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
h p
x
� 
�
%s
*synth2�
�|OTTER_MCU   | OTTER_MEMORY/memory_reg | 16 K x 32(READ_FIRST)  | W | R | 16 K x 32(WRITE_FIRST) |   | R | Port A and B     | 0      | 16     | 
h p
x
� 
�
%s
*synth2�
�+------------+-------------------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

h p
x
� 
A
%s
*synth2)
'
Distributed RAM: Final Mapping Report
h p
x
� 
-
%s
*synth2
-------NONE-------
h p
x
� 
�
%s
*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
3
%s
*synth2
Start Technology Mapping
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_02
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_02
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_12
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_12
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_22
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_22
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_32
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_32
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_42
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_42
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_52
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_52
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_62
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_62
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_72
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_72
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_82
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_82
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_92
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
OTTER_MEMORY/memory_reg_bram_92
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_102
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_102
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_112
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_112
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_122
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_122
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_132
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_132
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_142
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_142
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_152
BlockZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2!
OTTER_MEMORY/memory_reg_bram_152
BlockZ8-7052h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2~
|Finished Technology Mapping : Time (s): cpu = 00:00:51 ; elapsed = 00:00:53 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
-
%s
*synth2
Start IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
?
%s
*synth2'
%Start Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
B
%s
*synth2*
(Finished Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2x
vFinished IO Insertion : Time (s): cpu = 00:00:57 ; elapsed = 00:00:58 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Start Renaming Generated Instances
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:00:57 ; elapsed = 00:00:58 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start Rebuilding User Hierarchy
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Renaming Generated Ports
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Ports : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Start Renaming Generated Nets
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Nets : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Writing Synthesis Report
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
/
%s
*synth2

Report BlackBoxes: 
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
| |BlackBox name |Instances |
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
/
%s*synth2

Report Cell Usage: 
h px� 
4
%s*synth2
+------+---------+------+
h px� 
4
%s*synth2
|      |Cell     |Count |
h px� 
4
%s*synth2
+------+---------+------+
h px� 
4
%s*synth2
|1     |BUFG     |     1|
h px� 
4
%s*synth2
|2     |CARRY4   |    67|
h px� 
4
%s*synth2
|3     |LUT1     |    15|
h px� 
4
%s*synth2
|4     |LUT2     |   173|
h px� 
4
%s*synth2
|5     |LUT3     |   142|
h px� 
4
%s*synth2
|6     |LUT4     |   254|
h px� 
4
%s*synth2
|7     |LUT5     |   176|
h px� 
4
%s*synth2
|8     |LUT6     |   722|
h px� 
4
%s*synth2
|9     |MUXF7    |   128|
h px� 
4
%s*synth2
|10    |MUXF8    |    62|
h px� 
4
%s*synth2
|11    |RAM32M   |    10|
h px� 
4
%s*synth2
|12    |RAM32X1D |     4|
h px� 
4
%s*synth2
|13    |RAMB36E1 |    16|
h px� 
4
%s*synth2
|15    |FDRE     |   513|
h px� 
4
%s*synth2
|16    |IBUF     |    34|
h px� 
4
%s*synth2
|17    |OBUF     |     1|
h px� 
4
%s*synth2
|18    |OBUFT    |    64|
h px� 
4
%s*synth2
+------+---------+------+
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
a
%s
*synth2I
GSynthesis finished with 0 errors, 0 critical warnings and 98 warnings.
h p
x
� 
�
%s
*synth2�
Synthesis Optimization Runtime : Time (s): cpu = 00:00:47 ; elapsed = 00:00:57 . Memory (MB): peak = 1557.246 ; gain = 644.516
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Complete : Time (s): cpu = 00:00:57 ; elapsed = 00:00:59 . Memory (MB): peak = 1557.246 ; gain = 695.000
h p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0372

1569.1212
0.000Z17-268h px� 
U
-Analyzing %s Unisim elements for replacement
17*netlist2
287Z29-17h px� 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
R
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
12
14Z31-138h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0012

1572.8012
0.000Z17-268h px� 
�
!Unisim Transformation Summary:
%s111*project2�
�  A total of 14 instances were transformed.
  RAM32M => RAM32M (inverted pins: WCLK) (RAMD32(x6), RAMS32(x2)): 10 instances
  RAM32X1D => RAM32X1D (inverted pins: WCLK) (RAMD32(x2)): 4 instances
Z1-111h px� 
V
%Synth Design complete | Checksum: %s
562*	vivadotcl2

941b67beZ4-1430h px� 
C
Releasing license: %s
83*common2
	SynthesisZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1302
2512
902
0Z4-41h px� 
L
%s completed successfully
29*	vivadotcl2
synth_designZ4-42h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
synth_design: 2

00:01:032

00:01:052

1572.8012

1101.969Z17-268h px� 
c
%s6*runtcl2G
ESynthesis results are not added to the cache due to CRITICAL_WARNING
h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Write ShapeDB Complete: 2

00:00:002
00:00:00.0062

1572.8012
0.000Z17-268h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2c
aC:/Users/Advika Deodhar/Downloads/333OTTERPipeline/project_2/project_2.runs/synth_1/OTTER_MCU.dcpZ17-1381h px� 
�
%s4*runtcl2j
hExecuting : report_utilization -file OTTER_MCU_utilization_synth.rpt -pb OTTER_MCU_utilization_synth.pb
h px� 
\
Exiting %s at %s...
206*common2
Vivado2
Wed May 15 15:19:10 2024Z17-206h px� 


End Record