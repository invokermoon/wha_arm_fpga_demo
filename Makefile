program := arm_bin

arm_src_dir :=$(PWD)/src
arm_out_dir :=$(PWD)/out

$(info arm_out_dir=$(arm_out_dir))
$(info arm_src_dir=$(arm_src_dir))


SOURCES := $(wildcard $(arm_src_dir)/*.c)
SOURCES += $(wildcard $(arm_src_dir)/src/*.c)
#OBJS    := $(patsubst %.c,$(arm_out_dir)/%.o,$(SOURCES))
DIR_OBJS        = $(patsubst %.c,%.o,$(SOURCES))
OBJS_NAME        = $(notdir $(patsubst %.c,%.o,$(SOURCES)))
OBJS          = $(addprefix $(arm_out_dir)/,$(notdir $(patsubst %.c,%.o,$(SOURCES)))) 

$(info objs=$(OBJS))

CC      := gcc
CFLAGS  := -ggdb -Wall -DBUILD_TIME="\"`date`\"" -DDEBUG_
INCLUDE := -I ./
LIB     := -lpthread -ldl -ljpeg -lrt

.PHONY: clean install

$(program): $(OBJS)
	    $(CC) -o $@ $^ $(LIB)
$(arm_out_dir)/%.o: $(arm_src_dir)/%.c
	    mkdir -p $(arm_out_dir)
	    $(CC) -o $@ -c $< $(CFLAGS) $(INCLUDE)

clean:
	    rm $(OBJS) $(program) -f

#install: $(program)
#	    cp $(program) ./bin/
