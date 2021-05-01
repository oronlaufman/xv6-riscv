.global start_sigret_injection
.global end_sigret_injection

start_sigret_injection:
    li a7, 24
    ecall
    ret
end_sigret_injection: