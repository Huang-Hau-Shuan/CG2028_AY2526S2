/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb
 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Write Student 1’s Name here: ABCD (A1234567R)
@ Write Student 2’s Name here: WXYZ (A0000007X)
@ You could create a look-up table of registers here:
@ R0 ...
@ R1 ...
@ Register usage:
@ R0 = N (number of samples / divisor)
@ R1 = accel_buff (pointer to int array)
@ R2 = sum accumulator
@ R3 = loop counter i
@ R4 = temporary (loaded sample)
@ write your program from here:
mov_avg:
 PUSH {r2-r11, lr}

 MOV R2, #0              @ sum = 0
 MOV R3, #0              @ i = 0

loop:
 CMP R3, R0              @ compare i with N
 BGE done                @ if i >= N, exit loop
 LDR R4, [R1, R3, LSL #2] @ R4 = accel_buff[i]  (int = 4 bytes)
 ADD R2, R2, R4          @ sum += accel_buff[i]
 ADD R3, R3, #1          @ i++
 B loop                  @ repeat

done:
 SDIV R0, R2, R0         @ R0 = sum / N  (signed integer divide)

 POP {r2-r11, pc}
