/*
 * mov_avg.s
 *
 * CG2028 Assignment starter file.
 */
.syntax unified
.cpu cortex-m4
.thumb
.global ewma_filter
.type ewma_filter, %function

.text
.align 2

@ CG2028 Assignment
@ (c) ECE NUS
@ Write Student 1's Name here: ABCD (A1234567R)
@ Write Student 2's Name here: WXYZ (A0000007X)
@
@ Function prototype:
@   int ewma_filter(int new_data, int old_output, int alpha_percent);
@
@ ARM calling convention:
@   R0 = new_data       (signed integer sensor sample)
@   R1 = old_output     (previous filtered output)
@   R2 = alpha_percent  (integer from 0 to 100)
@   Return R0 = (alpha_percent * new_data
@                + (100 - alpha_percent) * old_output) / 100
@
@ Notes:
@ - Use signed integer arithmetic.
@ - Integer division must truncate towards zero, matching C integer division.
@ - Preserve all callee-saved registers that you use (R4-R11).
@ - Do not call a C helper function and do not use floating-point instructions.
@
@ Register table:
@   R0 ...
@   R1 ...
@   R2 ...
@   R3 ...
@   R4 ...
@
@ Write your program from here.
ewma_filter:
@    PUSH {r4-r7, lr}
@ eqn: output = [alpha_percent x new_data + (100 - alpha_percent) x old_output] / 100
@ diff = 100 - alpha_percent
@ term1 = alpha_percent × new_data
@ term2 = diff × old_output
@ numerator = term1 + term2
@ output = numerator / 100
    @ TODO: Implement the EWMA low-pass filter in pure ARM assembly.
    MOV R3, #100
    SUB R3, R2
    MUL R0, R2
    MUL R1, R3
    ADD R0, R1
    MOV R3, #100
    SDIV R0, R0, R3
    BX LR

@    POP  {r4-r7, pc}

.size ewma_filter, .-ewma_filter
