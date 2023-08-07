.global _startcode 
_startcode:
LDR sp, =_stack

/* Reset and start Global Timer */
mov	r0, #0x0
mov	r1, #0x0

bl init_ocm

BL main
B .
