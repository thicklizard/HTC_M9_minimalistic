prefix = /usr

CC = gcc

<<<<<<< HEAD
all : bpf_jit_disasm

bpf_jit_disasm : CFLAGS = -Wall -O2
bpf_jit_disasm : LDLIBS = -lopcodes -lbfd -ldl
bpf_jit_disasm : bpf_jit_disasm.o

=======
CFLAGS += -Wall -O2
CFLAGS += -D__EXPORTED_HEADERS__ -I../../include/uapi -I../../include

%.yacc.c: %.y
	$(YACC) -o $@ -d $<

%.lex.c: %.l
	$(LEX) -o $@ $<

all : bpf_jit_disasm bpf_dbg bpf_asm

bpf_jit_disasm : CFLAGS += -DPACKAGE='bpf_jit_disasm'
bpf_jit_disasm : LDLIBS = -lopcodes -lbfd -ldl
bpf_jit_disasm : bpf_jit_disasm.o

bpf_dbg : LDLIBS = -lreadline
bpf_dbg : bpf_dbg.o

bpf_asm : LDLIBS =
bpf_asm : bpf_asm.o bpf_exp.yacc.o bpf_exp.lex.o
bpf_exp.lex.o : bpf_exp.yacc.c

>>>>>>> 0e91d2a... Nougat
clean :
	rm -rf *.o bpf_jit_disasm

install :
	install bpf_jit_disasm $(prefix)/bin/bpf_jit_disasm
