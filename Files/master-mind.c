/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 *
 * Compile:
 *   gcc -c -o master-mind.o master-mind.c
 *   gcc -o master-mind master-mind.o
 * Run:
 *   sudo ./master-mind
 *
 * OR use the Makefile to build:
 *   > make all
 * and run:
 *   > make run
 * and test:
 *   > make test
 *
 ***********************************************************************
 * The Low-level interface to LED, button, and LCD is based on:
 *   wiringPi libraries by Gordon Henderson.
 * See:
 *   https://projects.drogon.net/raspberry-pi/wiringpi/
 */

#define _POSIX_C_SOURCE 200809L  /* For usleep, getopt, etc. */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <ctype.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
#define DEBUG
#undef ASM_CODE

/* ======================================================= */
/* Tunables & PIN definitions (using BCM numbering) */
// Adjust these as per your wiring.
#define LED         13    // Green LED – used for echo output
#define LED2        5     // Red LED – used for input feedback
#define BUTTON      19    // Button

/* Delays */
#define DELAY       200   // milliseconds
#define TIMEOUT     3000000  // microseconds

/* App constants */
#define COLS        3
#define SEQL        3

/* Generic constants */
#ifndef TRUE
#define TRUE (1==1)
#define FALSE (1==2)
#endif

#define PAGE_SIZE   (4*1024)
#define BLOCK_SIZE  (4*1024)
#define INPUT       0
#define OUTPUT      1
#define LOW         0
#define HIGH        1

/* ======================================================= */
/* LCD wiring definitions (using BCM numbering) */
#define STRB_PIN    24
#define RS_PIN      25
#define DATA0_PIN   23
#define DATA1_PIN   10
#define DATA2_PIN   27
#define DATA3_PIN   22

/* ======================================================= */
/* LCD command macros */
#define LCD_CLEAR       0x01
#define LCD_HOME        0x02
#define LCD_ENTRY       0x04
#define LCD_CTRL        0x08
#define LCD_CDSHIFT     0x10
#define LCD_FUNC        0x20
#define LCD_CGRAM       0x40
#define LCD_DGRAM       0x80

#define LCD_ENTRY_SH    0x01
#define LCD_ENTRY_ID    0x02

#define LCD_BLINK_CTRL  0x01
#define LCD_CURSOR_CTRL 0x02
#define LCD_DISPLAY_CTRL 0x04

#define LCD_FUNC_F      0x04
#define LCD_FUNC_N      0x08
#define LCD_FUNC_DL     0x10

#define LCD_CDSHIFT_RL  0x04

static unsigned int gpiobase;
static uint32_t *gpio;  /* This global pointer will be set via mmap */
#define PI_GPIO_MASK    (0xFFFFFFC0)

static int lcdControl = 0;
static int timed_out = 0;

/* LCD data structure */
struct lcdDataStruct {
    int bits, rows, cols;
    int rsPin, strbPin;
    int dataPins[8];
    int cx, cy;
};

/* Global game state and constants */
static const int colors = COLS;
static const int seqlen = SEQL;
static int *theSeq = NULL;
static int *seq1 = NULL, *seq2 = NULL, *cpy1 = NULL, *cpy2 = NULL;

/* --------------------------------------------------------------------------- */
/* Function Prototypes */
/* --------------------------------------------------------------------------- */

/* Utility functions */
int failure(int fatal, const char *message, ...);
void waitForEnter(void);
void delay(unsigned int howLong);
void delayMicroseconds(unsigned int howLong);

/* Low-level GPIO functions */
void digitalWrite(uint32_t *gpio, int pin, int value);
void pinMode(uint32_t *gpio, int pin, int mode);
void writeLED(uint32_t *gpio, int led, int value);
int readButton(uint32_t *gpio, int button);
int waitForButton(uint32_t *gpio, int button);

/* LCD functions */
void strobe(const struct lcdDataStruct *lcd);
void sendDataCmd(const struct lcdDataStruct *lcd, unsigned char data);
void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command);
void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command);
void lcdHome(struct lcdDataStruct *lcd);
void lcdClear(struct lcdDataStruct *lcd);
void lcdPosition(struct lcdDataStruct *lcd, int x, int y);
void lcdDisplay(struct lcdDataStruct *lcd, int state);
void lcdCursor(struct lcdDataStruct *lcd, int state);
void lcdCursorBlink(struct lcdDataStruct *lcd, int state);
void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data);
void lcdPuts(struct lcdDataStruct *lcd, const char *string);

/* Game logic functions */
void initSeq(void);
void showSeq(int *seq);
int countMatches(int *seq1, int *seq2);
void showMatches(int code, int *seq1, int *seq2, int lcd_format);
void readSeq(int *seq, int val);
int readNum(int max);

/* TIMER functions */
uint64_t timeInMicroseconds(void);
void timer_handler(int signum);
void initITimer(uint64_t timeout);

/* Blink function: blink an LED c times */
void blinkN(uint32_t *gpio, int led, int c);

/* Greeting function: blink based on surname letters (6 letters processed) */
void greetUser(uint32_t *gpio, int redLED, int greenLED, const char *surname);

/* Button press count helper */
int getButtonPressCount(uint32_t *gpio, int button, int durationSeconds);

/* --------------------------------------------------------------------------- */
/* Utility Function Definitions */
/* --------------------------------------------------------------------------- */

int failure(int fatal, const char *message, ...) {
    va_list argp;
    char buffer[1024];
    if (!fatal)
        return -1;
    va_start(argp, message);
    vsnprintf(buffer, 1023, message, argp);
    va_end(argp);
    fprintf(stderr, "%s", buffer);
    exit(EXIT_FAILURE);
    return 0;
}

void waitForEnter(void) {
    printf("Press ENTER to continue: ");
    (void)fgetc(stdin);
}

void delay(unsigned int howLong) {
    struct timespec sleeper, dummy;
    sleeper.tv_sec = (time_t)(howLong / 1000);
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;
    nanosleep(&sleeper, &dummy);
}

void delayMicroseconds(unsigned int howLong) {
    struct timespec sleeper;
    unsigned int uSecs = howLong % 1000000;
    unsigned int wSecs = howLong / 1000000;
    if (howLong == 0)
        return;
    else {
        sleeper.tv_sec = wSecs;
        sleeper.tv_nsec = (long)(uSecs * 1000L);
        nanosleep(&sleeper, NULL);
    }
}

/* --------------------------------------------------------------------------- */
/* Low-level GPIO Function Definitions */
/* --------------------------------------------------------------------------- */

void digitalWrite(uint32_t *gpio, int pin, int value) {
    int pin_offset = pin % 32;
    uint32_t pin_mask = 1 << pin_offset;
    if (value == HIGH) {
        gpio[7] = pin_mask;    // GPSET0 (offset 7)
    } else {
        gpio[10] = pin_mask;   // GPCLR0 (offset 10)
    }
}

void pinMode(uint32_t *gpio, int pin, int mode) {
    int reg_index = pin / 10;
    int offset = (pin % 10) * 3;
    uint32_t mask = 7 << offset;
    uint32_t reg = *(gpio + reg_index);
    reg = (reg & ~mask) | ((mode & 0x7) << offset);
    *(gpio + reg_index) = reg;
}

void writeLED(uint32_t *gpio, int led, int value) {
    pinMode(gpio, led, OUTPUT);
    digitalWrite(gpio, led, value);
}

int readButton(uint32_t *gpio, int button) {
    pinMode(gpio, button, INPUT);
    uint32_t level = gpio[13]; // Assume GPLEV0 is at offset 13
    int mask = 1 << button;
    return (level & mask) ? HIGH : LOW;
}

int waitForButton(uint32_t *gpio, int button) {
    while (1) {
        if (readButton(gpio, button) == HIGH) {
            delay(50);  // debounce delay
            if (readButton(gpio, button) == HIGH)
                return 1;
        }
        delay(10);
    }
}

int getButtonPressCount(uint32_t *gpio, int button, int durationSeconds) {
    int count = 0;
    time_t start = time(NULL);
    time_t end = start + durationSeconds;
    int prevState = readButton(gpio, button);
    while (time(NULL) < end) {
        int currState = readButton(gpio, button);
        if (currState == HIGH && prevState == LOW) {
            count++;
            while (readButton(gpio, button) == HIGH)
                delay(10);
        }
        prevState = currState;
        delay(10);
    }
    return count;
}

/* --------------------------------------------------------------------------- */
/* LCD Function Definitions */
/* --------------------------------------------------------------------------- */

void strobe(const struct lcdDataStruct *lcd) {
    digitalWrite(gpio, lcd->strbPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(gpio, lcd->strbPin, LOW);
    delayMicroseconds(50);
}

void sendDataCmd(const struct lcdDataStruct *lcd, unsigned char data) {
    unsigned char myData = data;
    unsigned char i, d4;
    if (lcd->bits == 4) {
        d4 = (myData >> 4) & 0x0F;
        for (i = 0; i < 4; i++) {
            digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
            d4 >>= 1;
        }
        strobe(lcd);
        d4 = myData & 0x0F;
        for (i = 0; i < 4; i++) {
            digitalWrite(gpio, lcd->dataPins[i], (d4 & 1));
            d4 >>= 1;
        }
    } else {
        for (i = 0; i < 8; i++) {
            digitalWrite(gpio, lcd->dataPins[i], (myData & 1));
            myData >>= 1;
        }
    }
    strobe(lcd);
}

void lcdPutCommand(const struct lcdDataStruct *lcd, unsigned char command) {
    digitalWrite(gpio, lcd->rsPin, LOW);
    sendDataCmd(lcd, command);
    delay(2);
}

void lcdPut4Command(const struct lcdDataStruct *lcd, unsigned char command) {
    unsigned char myCommand = command;
    unsigned char i;
    digitalWrite(gpio, lcd->rsPin, LOW);
    for (i = 0; i < 4; i++) {
        digitalWrite(gpio, lcd->dataPins[i], (myCommand & 1));
        myCommand >>= 1;
    }
    strobe(lcd);
}

void lcdHome(struct lcdDataStruct *lcd) {
    lcdPutCommand(lcd, LCD_HOME);
    lcd->cx = lcd->cy = 0;
    delay(5);
}

void lcdClear(struct lcdDataStruct *lcd) {
    lcdPutCommand(lcd, LCD_CLEAR);
    lcdPutCommand(lcd, LCD_HOME);
    lcd->cx = lcd->cy = 0;
    delay(5);
}

void lcdPosition(struct lcdDataStruct *lcd, int x, int y) {
    if (x > lcd->cols || x < 0)
        return;
    if (y > lcd->rows || y < 0)
        return;
    lcdPutCommand(lcd, x + (LCD_DGRAM | (y > 0 ? 0x40 : 0x00)));
    lcd->cx = x;
    lcd->cy = y;
}

void lcdDisplay(struct lcdDataStruct *lcd, int state) {
    if (state)
        lcdControl |= LCD_DISPLAY_CTRL;
    else
        lcdControl &= ~LCD_DISPLAY_CTRL;
    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursor(struct lcdDataStruct *lcd, int state) {
    if (state)
        lcdControl |= LCD_CURSOR_CTRL;
    else
        lcdControl &= ~LCD_CURSOR_CTRL;
    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdCursorBlink(struct lcdDataStruct *lcd, int state) {
    if (state)
        lcdControl |= LCD_BLINK_CTRL;
    else
        lcdControl &= ~LCD_BLINK_CTRL;
    lcdPutCommand(lcd, LCD_CTRL | lcdControl);
}

void lcdPutchar(struct lcdDataStruct *lcd, unsigned char data) {
    digitalWrite(gpio, lcd->rsPin, HIGH);
    sendDataCmd(lcd, data);
    if (++lcd->cx == lcd->cols) {
        lcd->cx = 0;
        if (++lcd->cy == lcd->rows)
            lcd->cy = 0;
        lcdPutCommand(lcd, lcd->cx + (LCD_DGRAM | (lcd->cy > 0 ? 0x40 : 0x00)));
    }
}

void lcdPuts(struct lcdDataStruct *lcd, const char *string) {
    while (*string)
        lcdPutchar(lcd, *string++);
}

/* --------------------------------------------------------------------------- */
/* Game Logic Functions */
/* --------------------------------------------------------------------------- */

void initSeq(void) {
    if (theSeq == NULL) {
        theSeq = (int *)malloc(SEQL * sizeof(int));
        if (theSeq == NULL) {
            fprintf(stderr, "Memory allocation for sequence failed\n");
            exit(EXIT_FAILURE);
        }
    }
    srand(time(NULL));
    for (int i = 0; i < SEQL; i++) {
        theSeq[i] = rand() % COLS + 1;
    }
}

void showSeq(int *seq) {
    printf("Secret sequence: ");
    for (int i = 0; i < SEQL; i++) {
        printf("%d ", seq[i]);
    }
    printf("\n");
}

int countMatches(int *seq1, int *seq2) {
    int exactMatches = 0;
    int approximateMatches = 0;
    int seq1Matched[SEQL] = {0};
    int seq2Matched[SEQL] = {0};
    for (int i = 0; i < SEQL; i++) {
        if (seq1[i] == seq2[i]) {
            exactMatches++;
            seq1Matched[i] = 1;
            seq2Matched[i] = 1;
        }
    }
    for (int i = 0; i < SEQL; i++) {
        if (!seq1Matched[i]) {
            for (int j = 0; j < SEQL; j++) {
                if (!seq2Matched[j] && seq1[i] == seq2[j]) {
                    approximateMatches++;
                    seq1Matched[i] = 1;
                    seq2Matched[j] = 1;
                    break;
                }
            }
        }
    }
    return exactMatches * 10 + approximateMatches;
}

void showMatches(int code, int *seq1, int *seq2, int lcd_format) {
    int approx = code % 10;
    int exact = (code - approx) / 10;
    printf("Exact: %d\nApproximate: %d\n", exact, approx);
}

void readSeq(int *seq, int val) {
    if (seq == NULL)
        seq = (int *)malloc(SEQL * sizeof(int));
    int first = val / 100;
    int second = (val % 100) / 10;
    int third = (val % 100) % 10;
    seq[0] = first;
    seq[1] = second;
    seq[2] = third;
}

int readNum(int max) {
    printf("Enter %d numbers:\n", SEQL);
    int inputs[SEQL];
    int counter = 0, input;
    while (counter < SEQL) {
        scanf("%d", &input);
        if (input > COLS || input < 1) {
            printf("Input must be between 1 and %d. Terminating\n", COLS);
            return 1;
        }
        inputs[counter] = input;
        counter++;
    }
    return 0;
}

/* --------------------------------------------------------------------------- */
/* TIMER Functions */
/* --------------------------------------------------------------------------- */

uint64_t timeInMicroseconds(void) {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return ((uint64_t)currentTime.tv_sec * 1000000) + currentTime.tv_usec;
}

void timer_handler(int signum) {
    uint64_t t = timeInMicroseconds();
    /* (Optional callback logic) */
}

void initITimer(uint64_t timeout) {
    struct itimerval timer;
    timer.it_value.tv_sec = timeout / 1000000;
    timer.it_value.tv_usec = timeout % 1000000;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    signal(SIGALRM, timer_handler);
    setitimer(ITIMER_REAL, &timer, NULL);
}

/* --------------------------------------------------------------------------- */
/* Blink Function: blink an LED c times */
/* --------------------------------------------------------------------------- */

void blinkN(uint32_t *gpio, int led, int c) {
    for (int i = 0; i < c; i++) {
        writeLED(gpio, led, HIGH);
        usleep(DELAY * 1000);
        writeLED(gpio, led, LOW);
        usleep(DELAY * 1000);
    }
}

/* --------------------------------------------------------------------------- */
/* Greeting Function: Blink based on surname letters (6 letters processed)
   For vowels, blink the green LED; for consonants, blink the red LED.
*/
void greetUser(uint32_t *gpio, int redLED, int greenLED, const char *surname) {
    for (int i = 0; i < 6 && surname[i] != '\0'; i++) {
        char c = tolower(surname[i]);
        if (c == 'a' || c == 'e' || c == 'i' || c == 'o' || c == 'u')
            blinkN(gpio, greenLED, 1);
        else
            blinkN(gpio, redLED, 1);
        delay(300);
    }
}

/* --------------------------------------------------------------------------- */
/* Main Function */
/* --------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {
    /* Set GPIO base address */
    gpiobase = 0x3F200000;
    struct lcdDataStruct *lcd;
    int bits, rows, cols;
    unsigned char func;
    int found = 0, attempts = 0, i, code;
    int *attSeq = NULL;
    int pinLED = LED;       // Green LED for echoing the button press count
    int pin2LED2 = LED2;    // Red LED for input feedback
    int pinButton = BUTTON;
    int fd;
    int exact;
    char buf[32];
    char str_in[20], dummyStr[20] = "some text";
    int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0, res_matches = 0;

    /* Process command-line arguments */
    {
        int opt;
        while ((opt = getopt(argc, argv, "hvdus:")) != -1) {
            switch(opt) {
                case 'v': verbose = 1; break;
                case 'h': help = 1; break;
                case 'd': debug = 1; break;
                case 'u': unit_test = 1; break;
                case 's': opt_s = atoi(optarg); break;
                default:
                    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]\n", argv[0]);
                    exit(EXIT_FAILURE);
            }
        }
    }
    if (help) {
        fprintf(stderr, "MasterMind program running on a Raspberry Pi with connected LED, button and LCD display\n");
        fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]\n", argv[0]);
        exit(EXIT_SUCCESS);
    }
    if (unit_test && optind >= argc - 1) {
        fprintf(stderr, "Expected 2 arguments after option -u\n");
        exit(EXIT_FAILURE);
    }
    if (verbose && unit_test) {
        printf("1st argument = %s\n", argv[optind]);
        printf("2nd argument = %s\n", argv[optind+1]);
    }
    if (verbose) {
        fprintf(stdout, "Settings: Verbose=%s, Debug=%s, Unittest=%s\n",
                verbose ? "ON" : "OFF", debug ? "ON" : "OFF", unit_test ? "ON" : "OFF");
        if (opt_s)
            fprintf(stdout, "Secret sequence set to %d\n", opt_s);
    }
    seq1 = (int *)malloc(seqlen * sizeof(int));
    seq2 = (int *)malloc(seqlen * sizeof(int));
    cpy1 = (int *)malloc(seqlen * sizeof(int));
    cpy2 = (int *)malloc(seqlen * sizeof(int));
    if (unit_test && argc > optind + 1) {
        strcpy(str_in, argv[optind]);
        opt_m = atoi(str_in);
        strcpy(str_in, argv[optind+1]);
        opt_n = atoi(str_in);
        readSeq(seq1, opt_m);
        readSeq(seq2, opt_n);
        if (verbose)
            fprintf(stdout, "Testing matches with sequences %d and %d\n", opt_m, opt_n);
        res_matches = countMatches(seq1, seq2);
        showMatches(res_matches, seq1, seq2, 1);
        exit(EXIT_SUCCESS);
    }
    if (opt_s) {
        if (theSeq == NULL)
            theSeq = (int *)malloc(seqlen * sizeof(int));
        readSeq(theSeq, opt_s);
        if (verbose) {
            fprintf(stderr, "Using secret sequence:\n");
            showSeq(theSeq);
        }
    }
    /* LCD configuration */
    bits = 4; cols = 16; rows = 2;
    printf("Raspberry Pi LCD driver, for a %dx%d display (%d-bit wiring)\n", cols, rows, bits);
    if (geteuid() != 0)
        fprintf(stderr, "setup: Must be root. (Did you forget sudo?)\n");

    attSeq = (int *)malloc(seqlen * sizeof(int));
    cpy1 = (int *)malloc(seqlen * sizeof(int));
    cpy2 = (int *)malloc(seqlen * sizeof(int));

    /* Open /dev/mem and map GPIO */
    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
        return failure(FALSE, "setup: Unable to open /dev/mem: %s\n", strerror(errno));
    gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpiobase);
    if ((intptr_t)gpio == -1)
        return failure(FALSE, "setup: mmap (GPIO) failed: %s\n", strerror(errno));

    /* Configure LED, Button, and LCD pins */
    pinMode(gpio, LED, OUTPUT);
    pinMode(gpio, LED2, OUTPUT);
    pinMode(gpio, BUTTON, INPUT);
    pinMode(gpio, STRB_PIN, OUTPUT);
    pinMode(gpio, RS_PIN, OUTPUT);
    pinMode(gpio, DATA0_PIN, OUTPUT);
    pinMode(gpio, DATA1_PIN, OUTPUT);
    pinMode(gpio, DATA2_PIN, OUTPUT);
    pinMode(gpio, DATA3_PIN, OUTPUT);

    /* Create and initialize LCD structure */
    lcd = (struct lcdDataStruct *)malloc(sizeof(struct lcdDataStruct));
    if (lcd == NULL)
        return -1;
    lcd->rsPin = RS_PIN;
    lcd->strbPin = STRB_PIN;
    lcd->bits = 4;
    lcd->rows = rows;
    lcd->cols = cols;
    lcd->cx = 0;
    lcd->cy = 0;
    lcd->dataPins[0] = DATA0_PIN;
    lcd->dataPins[1] = DATA1_PIN;
    lcd->dataPins[2] = DATA2_PIN;
    lcd->dataPins[3] = DATA3_PIN;

    digitalWrite(gpio, lcd->rsPin, LOW);
    pinMode(gpio, lcd->rsPin, OUTPUT);
    digitalWrite(gpio, lcd->strbPin, LOW);
    pinMode(gpio, lcd->strbPin, OUTPUT);
    for (i = 0; i < bits; i++) {
        digitalWrite(gpio, lcd->dataPins[i], LOW);
        pinMode(gpio, lcd->dataPins[i], OUTPUT);
    }
    delay(35);

    if (bits == 4) {
        func = LCD_FUNC | LCD_FUNC_DL;
        lcdPut4Command(lcd, func >> 4);
        delay(35);
        lcdPut4Command(lcd, func >> 4);
        delay(35);
        lcdPut4Command(lcd, func >> 4);
        delay(35);
        func = LCD_FUNC;
        lcdPut4Command(lcd, func >> 4);
        delay(35);
        lcd->bits = 4;
    } else {
        failure(TRUE, "setup: only 4-bit connection supported\n");
    }
    if (lcd->rows > 1) {
        func |= LCD_FUNC_N;
        lcdPutCommand(lcd, func);
        delay(35);
    }
    lcdDisplay(lcd, TRUE);
    lcdCursor(lcd, FALSE);
    lcdCursorBlink(lcd, FALSE);
    lcdClear(lcd);
    lcdPutCommand(lcd, LCD_ENTRY | LCD_ENTRY_ID);
    lcdPutCommand(lcd, LCD_CDSHIFT | LCD_CDSHIFT_RL);

    /* --- Greeting Section --- */
    fprintf(stderr, "Printing welcome message on the LCD display ...\n");
    lcdPuts(lcd, "Welcome to");
    lcdPosition(lcd, 1, 1);
    lcdPuts(lcd, "MasterMind");
    delay(2000);
    lcdClear(lcd);
    /* Blink greeting based on surname "Rafati" (6 letters) */
    greetUser(gpio, LED2, LED, "Rafati");
    lcdClear(lcd);
    lcdPuts(lcd, "Hello,");
    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, "Rafati");
    delay(2000);
    lcdClear(lcd);

    /* For testing, print secret sequence in terminal */
    if (!opt_s)
        initSeq();
    showSeq(theSeq);

    /* Start game when hardware button is pressed */
    lcdPuts(lcd, "Press button");
    lcdPosition(lcd, 0, 1);
    lcdPuts(lcd, "to start");
    while (!waitForButton(gpio, BUTTON)) {
        delay(10);
    }

    /* Game Round: exactly 3 turns */
    attSeq = (int *)malloc(seqlen * sizeof(int));
    if (attSeq == NULL)
        failure(TRUE, "Memory allocation error\n");

    printf("Round: 1\n");
    lcdPuts(lcd, "Round 1");
    delay(2000);

    /* For each turn, perform input collection */
    for (i = 0; i < SEQL; i++) {
        printf("Turn: %d\n", i + 1);
        /* Blink red LED only once at the start of the turn */
        writeLED(gpio, LED2, HIGH);
        delay(200);
        writeLED(gpio, LED2, LOW);
        delay(200);

        /* Collect button press count during a 5-second window */
        int buttonPressCount = getButtonPressCount(gpio, BUTTON, 5);
        printf("Button pressed %d times\n", buttonPressCount);
        /* Echo the count via green LED blinking */
        blinkN(gpio, LED, buttonPressCount);
        attSeq[i] = buttonPressCount;
        delay(500);
    }

    /* Evaluate the guess */
    int matches = countMatches(attSeq, theSeq);
    int approx = matches % 10;
    exact = (matches - approx) / 10;
    printf("Exact: %d\nApproximate: %d\n", exact, approx);
    delay(500);
    lcdClear(lcd);
    /* Display answer on LCD and via LED blinks */
    blinkN(gpio, LED, exact);
    sprintf(buf, "%d exact", exact);
    lcdPosition(lcd, 1, 0);
    lcdPuts(lcd, buf);
    blinkN(gpio, LED, approx);
    sprintf(buf, "%d approx", approx);
    lcdPosition(lcd, 1, 1);
    lcdPuts(lcd, buf);
    delay(1000);
    lcdClear(lcd);

    /* End-of-game outcome */
    if (exact == 3) {
        printf("SUCCESS\n");
        lcdPuts(lcd, "SUCCESS");
        usleep(500000);
        lcdPosition(lcd, 0, 1);
        sprintf(buf, "Attempts: %d", SEQL);
        lcdPuts(lcd, buf);
        writeLED(gpio, LED2, HIGH);
        blinkN(gpio, LED, 3);
        writeLED(gpio, LED2, LOW);
        lcdClear(lcd);
        lcdPuts(lcd, "Ending game");
        usleep(1000000);
        lcdClear(lcd);
        writeLED(gpio, LED2, 0);
    } else {
        lcdClear(lcd);
        printf("Sequence not found\n");
        lcdPuts(lcd, "GAME OVER");
        delay(5000);
        lcdClear(lcd);
        lcdPuts(lcd, "Ending game");
        usleep(1000000);
        lcdClear(lcd);
        writeLED(gpio, LED2, 0);
    }

    /* Free allocated memory */
    free(theSeq);
    free(seq1);
    free(seq2);
    free(cpy1);
    free(cpy2);
    free(attSeq);
    free(lcd);

    return 0;
}
