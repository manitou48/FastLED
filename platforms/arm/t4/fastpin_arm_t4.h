#ifndef __FASTPIN_ARM_T4_H
#define __FASTPIN_ARM_T4_H

FASTLED_NAMESPACE_BEGIN

#if defined(FASTLED_FORCE_SOFTWARE_PINS)
#warning "Software pin support forced, pin access will be slightly slower."
#define NO_HARDWARE_PIN_SUPPORT
#undef HAS_HARDWARE_PIN_SUPPORT

#else


/// Template definition for teensy 3.0 style ARM pins, providing direct access to the various GPIO registers.  Note that this
/// uses the full port GPIO registers.  In theory, in some way, bit-band register access -should- be faster, however I have found
/// that something about the way gcc does register allocation results in the bit-band code being slower.  It will need more fine tuning.
/// The registers are data output, set output, clear output, toggle output, input, and direction
template<uint8_t PIN, uint32_t _MASK, typename _DR, typename _PSOR, typename _PCOR, typename _PTOR,  typename _PDDR> class _ARMPIN {
public:
	typedef volatile uint32_t * port_ptr_t;
	typedef uint32_t port_t;

	inline static void setOutput() { pinMode(PIN, OUTPUT); } // TODO: perform MUX config { _PDDR::r() |= _MASK; }
	inline static void setInput() { pinMode(PIN, INPUT); } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }

	inline static void hi() __attribute__ ((always_inline)) { _PSOR::r() = _MASK; }
	inline static void lo() __attribute__ ((always_inline)) { _PCOR::r() = _MASK; }
	inline static void set(register port_t val) __attribute__ ((always_inline)) { _DR::r() = val; }

	inline static void strobe() __attribute__ ((always_inline)) { toggle(); toggle(); }

	inline static void toggle() __attribute__ ((always_inline)) { _PTOR::r() = _MASK; }

	inline static void hi(register port_ptr_t port) __attribute__ ((always_inline)) { hi(); }
	inline static void lo(register port_ptr_t port) __attribute__ ((always_inline)) { lo(); }
	inline static void fastset(register port_ptr_t port, register port_t val) __attribute__ ((always_inline)) { *port = val; }

	inline static port_t hival() __attribute__ ((always_inline)) { return _DR::r() | _MASK; }
	inline static port_t loval() __attribute__ ((always_inline)) { return _DR::r() & ~_MASK; }
	inline static port_ptr_t port() __attribute__ ((always_inline)) { return &_DR::r(); }
	inline static port_ptr_t sport() __attribute__ ((always_inline)) { return &_PSOR::r(); }
	inline static port_ptr_t cport() __attribute__ ((always_inline)) { return &_PCOR::r(); }
	inline static port_t mask() __attribute__ ((always_inline)) { return _MASK; }
};

/// Template definition for teensy 3.0 style ARM pins using bit banding, providing direct access to the various GPIO registers.  GCC
/// does a poor job of optimizing around these accesses so they are not being used just yet.
template<uint8_t PIN, int _BIT, typename _DR, typename _PSOR, typename _PCOR, typename _PTOR,  typename _PDDR> class _ARMPIN_BITBAND {
public:
	typedef volatile uint32_t * port_ptr_t;
	typedef uint32_t port_t;

	inline static void setOutput() { pinMode(PIN, OUTPUT); } // TODO: perform MUX config { _PDDR::r() |= _MASK; }
	inline static void setInput() { pinMode(PIN, INPUT); } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }

	inline static void hi() __attribute__ ((always_inline)) { *_DR::template rx<_BIT>() = 1; }
	inline static void lo() __attribute__ ((always_inline)) { *_DR::template rx<_BIT>() = 0; }
	inline static void set(register port_t val) __attribute__ ((always_inline)) { *_DR::template rx<_BIT>() = val; }

	inline static void strobe() __attribute__ ((always_inline)) { toggle(); toggle(); }

	inline static void toggle() __attribute__ ((always_inline)) { *_PTOR::template rx<_BIT>() = 1; }

	inline static void hi(register port_ptr_t port) __attribute__ ((always_inline)) { hi();  }
	inline static void lo(register port_ptr_t port) __attribute__ ((always_inline)) { lo(); }
	inline static void fastset(register port_ptr_t port, register port_t val) __attribute__ ((always_inline)) { *_DR::template rx<_BIT>() = val; }

	inline static port_t hival() __attribute__ ((always_inline)) { return 1; }
	inline static port_t loval() __attribute__ ((always_inline)) { return 0; }
	inline static port_ptr_t port() __attribute__ ((always_inline)) { return _DR::template rx<_BIT>(); }
	inline static port_t mask() __attribute__ ((always_inline)) { return 1; }
};

// Macros for k20 pin access/definition
#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))

#define _R(T) struct __gen_struct_ ## T
#define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline reg32_t r() { return T; } \
	template<int BIT> static __attribute__((always_inline)) inline ptr_reg32_t rx() { return GPIO_BITBAND_PTR(T, BIT); } };
#define _IO32(L) _RD32(GPIO ## L ## _DR); _RD32(GPIO ## L ## _DR_SET); _RD32(GPIO ## L ## _DR_CLEAR); _RD32(GPIO ## L ## _DR_TOGGLE);  _RD32(GPIO ## L ## _GDIR);

#define _DEFPIN_ARM(PIN, BIT, L) template<> class FastPin<PIN> : public _ARMPIN<PIN, 1 << BIT, _R(GPIO ## L ## _DR), _R(GPIO ## L ## _DR_SET), _R(GPIO ## L ## _DR_CLEAR), \
																			_R(GPIO ## L ## _DR_TOGGLE), _R(GPIO ## L ## _GDIR)> {}; \
									template<> class FastPinBB<PIN> : public _ARMPIN_BITBAND<PIN, BIT, _R(GPIO ## L ## _DR), _R(GPIO ## L ## _DR_SET), _R(GPIO ## L ## _DR_CLEAR), \
 																			_R(GPIO ## L ## _DR_TOGGLE),  _R(GPIO ## L ## _GDIR)> {};

// Actual pin definitions   T4 beta
#if defined(__IMXRT1052__)

_IO32(1); _IO32(2); _IO32(3); _IO32(4);

#define MAX_PIN 34
_DEFPIN_ARM( 0,  3, 1); _DEFPIN_ARM( 1,  2, 1); _DEFPIN_ARM( 2,  4, 4); _DEFPIN_ARM( 3,  5, 4);
_DEFPIN_ARM( 4,  6, 4); _DEFPIN_ARM( 5,  7, 4); _DEFPIN_ARM( 6, 17,  2); _DEFPIN_ARM( 7, 16, 2);
_DEFPIN_ARM( 8, 10, 2); _DEFPIN_ARM( 9, 11, 2); _DEFPIN_ARM(10,  0, 2); _DEFPIN_ARM(11,  2, 2);
_DEFPIN_ARM(12,  1, 2); _DEFPIN_ARM(13,  3, 2); _DEFPIN_ARM(14, 18, 1); _DEFPIN_ARM(15, 19, 1);
_DEFPIN_ARM(16, 23, 1); _DEFPIN_ARM(17, 22, 1); _DEFPIN_ARM(18, 17, 1); _DEFPIN_ARM(19, 16, 1);
_DEFPIN_ARM(20, 26, 1); _DEFPIN_ARM(21, 27, 1); _DEFPIN_ARM(22, 24, 1); _DEFPIN_ARM(23, 25, 1);
_DEFPIN_ARM(24, 12, 1); _DEFPIN_ARM(25, 13, 1); _DEFPIN_ARM(26, 30, 1); _DEFPIN_ARM(27, 31, 1);
_DEFPIN_ARM(28, 18, 3); _DEFPIN_ARM(29, 31, 4); _DEFPIN_ARM(30, 24, 4); _DEFPIN_ARM(31, 23, 4);
_DEFPIN_ARM(32, 12, 2); _DEFPIN_ARM(33, 8, 4);


#define SPI_DATA 11
#define SPI_CLOCK 13

#if 0
#define SPI2_DATA 7
#define SPI2_CLOCK 14
#endif
#define ARM_HARDWARE_SPI

//#define FASTLED_TEENSY3
#define HAS_HARDWARE_PIN_SUPPORT
#endif

#endif // FASTLED_FORCE_SOFTWARE_PINS

FASTLED_NAMESPACE_END

#endif // __FASTPIN_ARM_T4_H
