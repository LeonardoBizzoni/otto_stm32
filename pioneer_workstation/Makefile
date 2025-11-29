CCFLAGS = -I../pioneer_controller/Core/Inc			    \
	  -pedantic -Wall -Werror -Wextra			    \
	  -Wconversion -Wdouble-promotion			    \
	  -Wundef -Wcast-qual -Wmissing-declarations		    \
	  -Wredundant-decls -Wno-unused-function -Wno-override-init \
	  -Wno-unused-variable

debug: CCFLAGS += -O0 -g3 -ggdb -fsanitize=address,undefined,leak \
		   -fsanitize-trap -fstack-protector-strong
debug: main

release: CCFLAGS += -O3 -s -march=native -flto -D_FORTIFY_SOURCE=2
release: main

main: src/main.c
	$(CC) -std=gnu11 $^ -o $@ $(CCFLAGS)
