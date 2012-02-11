/* General-purpose IO pins driver.
 *
 * Hugo Vincent, 2 August 2010.
 */

#ifndef GPIO_h
#define GPIO_h

// FIXME register pins, set direction, register interrupt

void GPIO_Init();
void GPIO_SetDirection(int block, unsigned int in_bitmap, unsigned int out_bitmap);
void GPIO_Write(int block, unsigned int set_bitmap, unsigned int clear_bitmap);
void GPIO_Toggle(int block, unsigned int bitmap);
unsigned int GPIO_Read(int block);

// Convenience Wrappers:
inline void GPIO_PinSet(int block, int pin) { GPIO_Write(block, 0x1<<pin, 0); }
inline int  GPIO_PinRead(int block, int pin) { return GPIO_Read(block) & pin ? 1 : 0; }
inline void GPIO_PinClear(int block, int pin) { GPIO_Write(block, 0, 0x1<<pin); }
inline void GPIO_PinToggle(int block, int pin) { GPIO_Toggle(block, 0x1<<pin); }
inline void GPIO_PinWrite(int block, int pin, int value) { if (value) \
	GPIO_PinSet(block, pin); else GPIO_PinClear(block, pin); }

#endif // ifndef GPIO_h

