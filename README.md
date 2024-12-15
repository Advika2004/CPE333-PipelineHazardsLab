CPE 333: Computer Hardware Architecture and Design Lab 2: Pipelining and Implementing Hazards

Executive Summary:
This project focused on implementing a pipelined version of the OTTER RISC-V CPU, emphasizing the handling of control, data, and load-use hazards. By splitting the memory module into instruction and data memory, we optimized the instruction fetch and execution stages. Additionally, custom modules like the Branch Hazard Module, ForwardingUnit, and HazardDetectorAndStaller were designed to handle hazards efficiently. With these enhancements, the pipelined OTTER achieved a 23% improvement in runtime compared to the non-pipelined version, as verified through benchmarks. Full details, schematics, and performance proofs are available in the lab report linked below.

Summary of Design:
The design followed a standard five-stage pipeline with additional optimizations for handling hazards and improving data flow:

Memory Splitting and Fetch Stage Optimization:
- Instruction memory was incorporated as part of the fetch-to-decode pipeline register, while data memory remained in the memory stage.
- A struct-based approach was used to group signals and propagate them through the pipeline.

Branch Handling:
- Branch detection logic was moved to the Branch Condition Generator in the execute stage, simplifying the decode stage.
- A new signal, branch_taken, identified branching instructions based on their func3 codes and served as input to the Branch Hazard Module.

Control Hazards:
- Implemented static branch prediction (default: branch not taken).
- The Branch Hazard Module used the branch_taken signal to trigger flush signals (fetch2dec_flush and dec2exec_flush), which cleared relevant pipeline registers to handle incorrect branch predictions.

Data Hazards and Forwarding:
- The ForwardingUnit compared rs1 and rs2 from decode-to-execute with destination registers from memory and writeback stages.
- Forwarding signals (FORWARDA and FORWARDB) controlled new muxes before the ALU, allowing selection between alu_result, writeback_out, or source registers.

Load-Use Hazards:
- The HazardDetectorAndStaller module monitored signals from fetch-to-decode and decode-to-execute stages to identify load-use hazards.
- This module stalled the pipeline by halting pc_write and disabling writes to the fetch-to-decode pipeline register, while activating a loadUseFlush signal to clear affected stages.

FSM and Controller Design:
- The FSM managed read and write enable signals for the cache, memory, and CLA modules.
- Additional muxes were added for address and data selection, with their control managed in the FSM.

Performance Comparison:
- Benchmarks using the testall file without hazards showed a significant efficiency improvement.
- The original OTTER took 205,410 ns, while the pipelined OTTER with hazards completed the same task in 157,930 ns, achieving a 23% reduction in runtime.

Full Lab Report with Schematics and Testing Simulations: https://docs.google.com/document/d/1B7yHBPtDDNcpNUUS0AoQ5Vz8yTgp9qTHKwWXnAVRvVM/edit?usp=sharing
