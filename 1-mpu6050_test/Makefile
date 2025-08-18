TARGET = mpu6050_test
SRC    = mpu6050_test.c
CFLAGS = -O2 -Wall

all: $(TARGET)

$(TARGET) : $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)