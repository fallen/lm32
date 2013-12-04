.include "macros.inc"
.text
.global _start

_start:
_reset_handler:
	xor r0, r0, r0
	mvhi r1, hi(_start)
	ori r1, r1, lo(_start)
	wcsr eba, r1
	wcsr deba, r1
	xor	r25, r25, r25
	bi _main
	nop

_breakpoint_handler:
	ori r25, r25, 1
	addi ba, ba, 4
	bret
	nop
	nop
	nop
	nop
	nop

_instruction_bus_error_handler:
	ori r25, r25, 2
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop

_watchpoint_handler:
	ori r25, r25, 4
	addi ba, ba, 4
	bret
	nop
	nop
	nop
	nop
	nop

_data_bus_error_handler:
	ori r25, r25, 8
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop

_divide_by_zero_handler:
	ori r25, r25, 16
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop

_interrupt_handler:
	ori r25, r25, 32
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop

_system_call_handler:
	ori r25, r25, 64
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop

_itlb_miss_handler:
	ori r25, r25, 128
	rcsr r24, TLBVADDR
	eret
	nop
	nop
	nop
	nop
	nop

_dtlb_miss_handler:
	ori r25, r25, 256
	rcsr r24, TLBVADDR
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop

_dtlb_fault_handler:
	ori r25, r25, 512
	rcsr r24, TLBVADDR
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop

_privilege_exception_handler:
	ori r25, r25, 1024
	addi ea, ea, 4
	eret
	nop
	nop
	nop
	nop
	nop
