#define SC_KEY_1 0x1e // Keyboard 1 and !
#define SC_KEY_2 0x1f // Keyboard 2 and @
#define SC_KEY_3 0x20 // Keyboard 3 and #
#define SC_KEY_4 0x21 // Keyboard 4 and $
#define SC_KEY_5 0x22 // Keyboard 5 and %
#define SC_KEY_6 0x23 // Keyboard 6 and ^
#define SC_KEY_7 0x24 // Keyboard 7 and &
#define SC_KEY_8 0x25 // Keyboard 8 and *
#define SC_KEY_9 0x26 // Keyboard 9 and (
#define SC_KEY_0 0x27 // Keyboard 0 and )
#define SC_KEY_BACKSPACE 0x2a
#define SC_KEY_KPASTERISK 0x55
#define SC_KEY_HASHTILDE 0x32
#define SC_KEY_DOWN 0x51 // Keyboard Down Arrow
#define SC_KEY_UP 0x52 // Keyboard Up Arrow
#define SC_KEY_ENTER 0x28



typedef enum{
	KNONE = 0,
	K0 = SC_KEY_0,
	K1 = SC_KEY_1,
	K2 = SC_KEY_2,
	K3 = SC_KEY_3,
	K4 = SC_KEY_4,
	K5 = SC_KEY_5,
	K6 = SC_KEY_6,
	K7 = SC_KEY_7,
	K8 = SC_KEY_8,
	K9 = SC_KEY_9,
	KSTAR = SC_KEY_KPASTERISK,
	KPOUND = SC_KEY_HASHTILDE,
	KC = SC_KEY_BACKSPACE,
	KMENU = SC_KEY_ENTER,
	KUP = SC_KEY_UP,
	KDOWN = SC_KEY_DOWN
	} nokia_key;



void keypad_init(void (*cb)(nokia_key));


