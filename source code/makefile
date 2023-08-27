# 定义 BIN/INC/LIB 路径
BIN := bin
INC := inc
LIB := lib

# 添加模块
include common/common.mk
include log/log.mk
include param_config/param_config.mk

# 增加 inc 路径
CFLAGS += -I$(INC)

# 增加 lib / bin 路径
LDFLAGS += -L$(LIB) -L$(BIN)

# 设置链接选项
LIBS += -l$(COMMON) \
		-l$(LOG) \
		-l$(PARAM_CONFIG) \
		-lpthread \
		-lrt

# 自定义宏: DEBUG
USER_MACROS += -DDEBUG

# 可执行文件主函数编译
ECU1_OBJ := ecu1.o
ECU2_OBJ := ecu2.o
ECU3_OBJ := ecu3.o
ECU4_OBJ := ecu4.o
ECU5_OBJ := ecu5.o

.PHONY: clean

# 编译所有
all: $(COMMON) $(LOG) $(PARAM_CONFIG) ecu1 ecu2 ecu3 ecu4 ecu5 

# 编译可执行文件
ecu1 : $(ECU1_OBJ)
	g++ -Wl,-rpath=. $(LDFLAGS) -o $(BIN)/$@ $^ $(LIBS)
ecu2 : $(ECU2_OBJ)
	g++ -Wl,-rpath=. $(LDFLAGS) -o $(BIN)/$@ $^ $(LIBS)
ecu3 : $(ECU3_OBJ)
	g++ -Wl,-rpath=. $(LDFLAGS) -o $(BIN)/$@ $^ $(LIBS)	
ecu4 : $(ECU4_OBJ)
	g++ -Wl,-rpath=. $(LDFLAGS) -o $(BIN)/$@ $^ $(LIBS)
ecu5 : $(ECU5_OBJ)
	g++ -Wl,-rpath=. $(LDFLAGS) -o $(BIN)/$@ $^ $(LIBS)

# 编译模块动态库
$(LOG): $(LOG_OBJ)
	g++ -shared -o $(BIN)/lib$@.so $^
$(COMMON): $(COMMON_OBJ)
	g++ -shared -o $(BIN)/lib$@.so $^
$(PARAM_CONFIG): $(PARAM_CONFIG_OBJ)
	g++ -shared -o $(BIN)/lib$@.so $^

# 编译 .o 文件
%.o: %.c
	g++ -c -std=c++11 -g -Wall $(CFLAGS) $(CPPFLAGS) $(USER_MACROS) -fPIC -o $@ $<

%.o: %.cpp
	g++ -c -std=c++11 -g -Wall $(CFLAGS) $(CPPFLAGS) $(USER_MACROS) -fPIC -o $@ $<


# 删除 .o 文件
# 删除动态库和可执行文件
clean:
	@rm -rf *.o
	@rm -rf bin/*

