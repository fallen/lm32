.include "macros.inc"

.macro check_psw val
	rcsr r5, PSW
	calli disable_dtlb
	calli disable_itlb
	mvhi r13, hi(\val)
	ori r13, r13, lo(\val)
	be r5, r13, 991f
	tc_fail
	bi 992f
991:
	tc_pass
992:
.endm

initial_page:
start

# This code is shared between QEMU and the milkymist test bench. In the
# latter the following placeholder will be replaced by a variable count of
# NOPs. The idea is to move the code around, so that a restart due to a
# instruction cache miss will be at different places.
##NOPS##

mvhi r1, hi(data2)
ori r1, r1, lo(data2)
mvhi r2, hi(data1)
ori r2, r2, lo(data1)
calli update_dtlb

test_name MMU_DTLB_OFF
    lw r3, (r1+0)
    check_r3 0xc0cac01a

test_name MMU_DTLB_ON
    calli enable_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_r3 0xdeadbabe

test_name MMU_DTLB_ASID_MATCH_0
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli set_asid_5
    calli enable_dtlb
1:
    lw r3, (r1+0)
    calli disable_dtlb
    check_r3 0xdeadbabe
    calli update_dtlb
    calli set_asid_0

test_name MMU_DTLB_ASID_MATCH_NOEXCP
    check_noexcp


test_name MMU_DTLB_ASID_MATCH_PSW_0
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli set_asid_5
    calli enable_dtlb
1:
    lw r3, (r1+0)
    check_psw 0x5040 # this reads PSW and disables DTLB and ITLB
    calli update_dtlb
    calli set_asid_0

test_name MMU_DTLB_ASID_MATCH_PSW_1
    check_r3 0xdeadbabe

test_name MMU_DTLB_ASID_MATCH_PSW_2
    check_noexcp


test_name MMU_DTLB_ASID_MISMATCH_PSW_1
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli enable_dtlb
    lw r3, (r1+0)
1:
    check_psw 0xc0 # this reads PSW and disables DTLB and ITLB
    calli update_dtlb

test_name MMU_DTLB_ASID_MISMATCH_PSW_2
    check_excp 256

test_name MMU_DTLB_ASID_MISMATCH_PSW_3
    check_reg ea 1b

test_name MMU_DTLB_ASID_MISMATCH_PSW_4
    xor r3, r3, r3
    ori r3, r3, 1
    wcsr IE, r3
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli enable_dtlb
    lw r3, (r1+0)
1:
    check_psw 0xc3 # this reads PSW and disables DTLB and ITLB
    calli update_dtlb

test_name MMU_DTLB_ASID_MISMATCH_PSW_5
    check_excp 256

test_name MMU_DTLB_ASID_MISMATCH_PSW_6
    check_reg ea 1b

test_name MMU_DTLB_ASID_MISSMATCH
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli set_asid_0
    calli enable_dtlb
    lw r3, (r1+0)
1:
    calli disable_dtlb
    calli update_dtlb
    check_excp 256

test_name MMU_DTLB_ASID_MISSMATCH_1
    check_reg ea 1b

test_name MMU_DTLB_ASID_MISSMATCH_2
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli set_asid_5
    calli enable_dtlb
    rcsr	r4, PSW
    mvhi	r5, hi(0xFFFE0FFF)
    ori		r5, r5, lo(0xFFFE0FFF)
    and		r4, r4, r5
    wcsr	PSW, r4 # set current ASID back to 0 right before doing the load
    lw r3, (r1+0)
1:
    calli disable_dtlb
    calli update_dtlb
    calli set_asid_0
    check_excp 256

test_name MMU_DTLB_ASID_MISSMATCH_3
    check_reg ea 1b

test_name MMU_DTLB_ASID_FLUSH_ASID_5
    xor r3, r3, r3
    calli flush_dtlb
    calli update_dtlb_with_asid_5
    calli set_asid_5
    calli enable_dtlb
    calli flush_dtlb_asid_5
    lw r3, (r1+0)
1:
    calli disable_dtlb
    calli update_dtlb
    calli set_asid_0
    check_excp 256

test_name MMU_DTLB_ASID_FLUSH_ASID_5_1
    check_reg ea 1b

test_name MMU_DTLB_ASID_MISS_0
    calli set_asid_5
    calli enable_dtlb
    lb r3, (r1+0x3fff)
1:
    calli disable_dtlb
    calli set_asid_0
    check_excp 256

test_name MMU_DTLB_ASID_MISS_1
    check_reg ea 1b

test_name MMU_DTLB_ASID_MISS_2
    check_reg r24 data2+0x3000+1  # page mask already applied

test_name MMU_DTLB_MISS_1
    calli enable_dtlb
    lb r3, (r1+0x1fff)
1:
    calli disable_dtlb
    check_excp 256

test_name MMU_DTLB_MISS_2
    check_reg ea 1b

test_name MMU_DTLB_MISS_3
    check_reg r24 data2+0x1000+1  # page mask already applied

test_name MMU_DTLB_MISS_4
    calli enable_dtlb
    lw r3, (r1+0x1004)
    lw r3, (r1+0x1004)
    calli disable_dtlb
    check_excp 256

test_name MMU_DTLB_MISS_5
    calli enable_dtlb
    mvi r3, 0x1234
    sw (r1+4), r3
    lw r3, (r1+0x1004)
    calli disable_dtlb
    check_excp 256

test_name MMU_DTLB_MISS_6
    lw r3, (r2+4)
    check_r3 0x1234

test_name MMU_DTLB_UPDATE_1
    calli flush_dtlb
    calli enable_dtlb
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11       # TLB entry setup
    lw r3, (r1+0)            # .. directly followed by a load
    calli disable_dtlb
    check_r3 0xdeadbabe

test_name MMU_DTLB_UPDATE_2
    calli flush_dtlb
    calli enable_dtlb
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    lw r3, (r1+0)            # A load
    wcsr TLBPADDR, r11       # .. directly followed an TLB update
    calli disable_dtlb
    check_excp 256

test_name MMU_DTLB_FAULT_1
    calli flush_dtlb
    mvhi r1, hi(data2)
    ori r1, r1, lo(data2)
    mvhi r2, hi(data1)
    ori r2, r2, lo(data1)
    calli update_dtlb_ro
    mv r3, r0
    calli enable_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_r3 0xdeadbabe

test_name MMU_DTLB_FAULT_2
    calli enable_dtlb
    sw (r1+0), r0
1:
    calli disable_dtlb
    check_excp 512

test_name MMU_DTLB_FAULT_3
    check_reg ea 1b

test_name MMU_DTLB_FAULT_4
    check_reg r24 data2+1    # page mask already applied

test_name MMU_DTLB_CACHE_INHIBIT_1
    mvhi r3, 0xaa55          # this is a test for the test below :)
    ori r3, r3, 0xaa55
    sw (r2+4), r3
    calli flush_dtlb
    mvhi r1, hi(data2)
    ori r1, r1, lo(data2)
    mvhi r2, hi(data1)
    ori r2, r2, lo(data1)
    calli update_dtlb
    calli enable_dtlb
    lw r3, (r1+4)
    calli disable_dtlb

    mvhi r3, 0x0010          # warning: dragons ahead
    add r2, r2, r3           # due to memory wraparound, the new [r2] will
    mvhi r3, 0x1234          # be an alias to the original one, but will
    ori r3, r3, 0x5678       # index another cache line
    sw (r2+4), r3

    calli enable_dtlb
    lw r3, (r1+4)
    calli disable_dtlb
    check_r3 0xaa55aa55      # because the value is fetched from cache

test_name MMU_DTLB_CACHE_INHIBIT_2
    mvhi r3, 0xaa55          # this is the same test as above, but the
    ori r3, r3, 0xaa55       # tlb entry has the cache inhibit flag set
    sw (r2+4), r3
    calli flush_dtlb
    mvhi r1, hi(data2)
    ori r1, r1, lo(data2)
    mvhi r2, hi(data1)
    ori r2, r2, lo(data1)
    calli update_dtlb_nocache
    calli enable_dtlb
    lw r3, (r1+4)
    calli disable_dtlb

    mvhi r3, 0x0010
    add r2, r2, r3
    mvhi r3, 0x1234
    ori r3, r3, 0x5678
    sw (r2+4), r3

    calli enable_dtlb
    lw r3, (r1+4)
    calli disable_dtlb
    check_r3 0x12345678      # .. therefore we expect the uncached value

test_name MMU_DTLB_FLUSH
    calli enable_dtlb
    calli flush_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_excp 256

test_name MMU_DTLB_INV
    mvhi r1, hi(data2)
    ori r1, r1, lo(data2)
    mvhi r2, hi(data1)
    ori r2, r2, lo(data1)
    calli update_dtlb

    calli enable_dtlb
    calli invalidate_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_excp 256


# make sure we have a mapping for the current code
mvhi r1, hi(initial_page)
ori r1, r1, lo(initial_page)
mv r2, r1
calli update_itlb

mvhi r1, hi(code2)
ori r1, r1, lo(code2)
mvhi r2, hi(code1)
ori r2, r2, lo(code1)
calli update_itlb

test_name MMU_ITLB_OFF
    calli flush_icache
    calli code2
    check_r3 0x55

test_name MMU_ITLB_ON
    calli flush_icache
    calli enable_itlb
    calli code2
    check_r3 0xaa

test_name MMU_ITLB_CACHE_ALIAS
    calli flush_icache
    calli code2
    calli enable_itlb
    calli code2
    check_r3 0xaa

test_name MMU_ITLB_MISS_1
    calli enable_itlb
    calli code2+0x1ffc
    check_excp 128

test_name MMU_ITLB_MISS_2
    check_reg ea code2+0x1ffc

test_name MMU_ITLB_MISS_3
    check_reg r24 code2+0x1000    # page mask already applied

test_name MMU_ITLB_MISS_4
    calli flush_dcache
    calli enable_itlb
    lw r1, (r0+0)            # provoke data cache restart request
    calli code2+0x1004
    check_excp 128

test_name MMU_ITLB_MISS_5
    calli flush_icache
    calli enable_itlb
    calli code2+0x1004      # provoke instruction cache restart request
    check_excp 128

test_name MMU_ITLB_INV
    mvhi r1, hi(code2)
    ori r1, r1, lo(code2)
    calli flush_icache
    calli enable_itlb
    calli invalidate_itlb
    mvhi ra, hi(1f)
    ori ra, ra, lo(1f)
    bi code2
1:
    check_excp 128

test_name MMU_ITLB_FLUSH
    calli flush_itlb
    # reload testcode mapping again
    mvhi r1, hi(initial_page)
    ori r1, r1, lo(initial_page)
    mv r2, r1
    calli update_itlb

    calli enable_itlb
    mvhi ra, hi(1f)
    ori ra, ra, lo(1f)
    bi code1
1:
    check_excp 128

test_name MMU_ITLB_EXCEPTION_1
    calli enable_itlb
    scall
    calli disable_itlb
    check_excp 64

test_name MMU_USER_MODE_1
    mvi r3, 0x200
    wcsr PSW, r3
1:
    wcsr PSW, r0
    check_excp 1024

test_name MMU_USER_MODE_2
    check_reg ea 1b

test_name MMU_USER_MODE_3
    check_csr PSW 0x400

# eusr <- usr
test_name MMU_USER_MODE_4
    mvi r3, 0x400
    wcsr PSW, r3
    mvhi ea, hi(1f)
    ori ea, ea, lo(1f)
    eret
1:
    rcsr r3, PSW
    calli disable_user_mode
    check_r3 0x600

# busr <- usr
test_name MMU_USER_MODE_5
    mvi r3, 0x800
    wcsr PSW, r3
    mvhi ba, hi(1f)
    ori ba, ba, lo(1f)
    bret
1:
    rcsr r3, PSW
    calli disable_user_mode
    check_r3 0xa00

end

enable_itlb:
    rcsr r10, PSW
    ori r10, r10, 0x8
    wcsr PSW, r10
    nop
    nop
    nop
    ret

disable_itlb:
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

enable_user_mode:
    rcsr r10, PSW
    ori r10, r10, 0x200
    wcsr PSW, r10
    nop
    nop
    nop
    ret

disable_user_mode:
    mv r10, ra
    scall
    mv ra, r10
    ret

flush_itlb:
    mvi r10, 2
    wcsr TLBVADDR, r10
    ret

update_itlb:
    wcsr TLBVADDR, r1
    wcsr TLBPADDR, r2
    ret

invalidate_itlb:
    ori r10, r1, 4
    wcsr TLBVADDR, r10
    ret

enable_dtlb:
    rcsr r10, PSW
    ori r10, r10, 0x40
    wcsr PSW, r10
    ret

disable_dtlb:
    rcsr r10, PSW
    andi r10, r10, 0xffbf
    wcsr PSW, r10
    ret

flush_dtlb:
    mvi r10, 3
    wcsr TLBVADDR, r10
    ret

update_dtlb:
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11
    ret

update_dtlb_with_asid_5:
    ori r10, r1, 1
    ori r11, r2, 1
    mvhi r5, hi(0xFFFFF07F)
    ori r5, r5, lo(0xFFFFF07F)
    and	r10, r5, r10
    ori r10, r10, 0x280 # ASID 5
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11
    ret

update_dtlb_ro:
    ori r10, r1, 1
    ori r11, r2, 3
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11
    ret

update_dtlb_nocache:
    ori r10, r1, 1
    ori r11, r2, 5
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11
    ret

invalidate_dtlb:
    ori r10, r1, 5
    wcsr TLBVADDR, r10
    ret

flush_dcache:
    wcsr DCC, r0
    ret

flush_icache:
    wcsr ICC, r0
    nop
    nop
    nop
    nop
    ret

set_asid_5:
    rcsr	r4, PSW
    mvhi	r5, hi(0xFFFE0FFF)
    ori		r5, r5, lo(0xFFFE0FFF)
    and		r4, r4, r5
    ori		r4, r4, 0x5000
    wcsr	PSW, r4
    ret

set_asid_0:
    rcsr	r4, PSW
    mvhi	r5, hi(0xFFFE0FFF)
    ori		r5, r5, lo(0xFFFE0FFF)
    and		r4, r4, r5
    wcsr	PSW, r4
    ret

flush_dtlb_asid_5:
    xor r5, r5, r5
    ori r5, r5, 0x7 # TLB_OP_FLUSH_ASID bits + DTLB bit set
    ori r5, r5, 0x280 # ASID 5
    wcsr TLBVADDR, r5
    ret

.align 0x1000
code1:
    mvi r3, 0xaa
    # disable itlb
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

.align 0x1000
code2:
    mvi r3, 0x55
    # disable itlb
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

.data
.align 0x1000
data1: .long 0xdeadbabe
.align 0x1000
data2: .long 0xc0cac01a
