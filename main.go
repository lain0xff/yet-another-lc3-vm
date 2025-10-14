package main

import (
	"encoding/binary"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"golang.org/x/sys/unix"
)

const (
	// Registers
	R_R0 = iota
	R_R1
	R_R2
	R_R3
	R_R4
	R_R5
	R_R6
	R_R7
	R_PC    // program counter
	R_COND  // condition flags
	R_COUNT // count of registers
)

const (
	// Condition Flags
	FL_POS = 1 << 0 // P
	FL_ZRO = 1 << 1 // Z
	FL_NEG = 1 << 2 // N
)

const (
	// Op Codes
	OP_BR = iota // branch
	OP_ADD       // add
	OP_LD        // load
	OP_ST        // store
	OP_JSR       // jump register
	OP_AND       // bitwise and
	OP_LDR       // load register
	OP_STR       // store register
	OP_RTI       // unused
	OP_NOT       // bitwise not
	OP_LDI       // load indirect
	OP_STI       // store indirect
	OP_JMP       // jump
	OP_RES       // reserved (unused)
	OP_LEA       // load effective address
	OP_TRAP      // execute trap
)

const (
	// Memory Mapped Registers
	MR_KBSR = 0xFE00 // keyboard status
	MR_KBDR = 0xFE02 // keyboard data
)

const (
	// Trap Codes
	TRAP_GETC  = 0x20 // get character from keyboard, not echoed onto the terminal
	TRAP_OUT   = 0x21 // output a character
	TRAP_PUTS  = 0x22 // output a word string
	TRAP_IN    = 0x23 // get character from keyboard, echoed onto the terminal
	TRAP_PUTSP = 0x24 // output a byte string
	TRAP_HALT  = 0x25 // halt the program
)

const (
	MEMORY_MAX = 1 << 16 // 65536 locations
	PC_START   = 0x3000  // default starting position
)

var (
	memory [MEMORY_MAX]uint16 // memory
	reg    [R_COUNT]uint16    // registers
)

var originalTermios unix.Termios

// Disable input buffering
func disableInputBuffering() {
	termios, err := unix.IoctlGetTermios(int(os.Stdin.Fd()), unix.TCGETS)
	if err != nil {
		panic(err)
	}
	originalTermios = *termios
	newTermios := *termios
	newTermios.Lflag &^= unix.ICANON | unix.ECHO
	if err := unix.IoctlSetTermios(int(os.Stdin.Fd()), unix.TCSETS, &newTermios); err != nil {
		panic(err)
	}
}

// Restore input buffering
func restoreInputBuffering() {
	if err := unix.IoctlSetTermios(int(os.Stdin.Fd()), unix.TCSETS, &originalTermios); err != nil {
		panic(err)
	}
}

// Check if a key is pressed
func checkKey() bool {
	var readfds unix.FdSet
	fd := int(os.Stdin.Fd())
	readfds.Set(fd)

	timeout := unix.Timeval{
		Sec:  0,
		Usec: 0,
	}

	n, err := unix.Select(fd+1, &readfds, nil, nil, &timeout)
	if err != nil {
		panic(err)
	}
	return n != 0
}

// Handle interrupt
func handleInterrupt() {
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-c
		restoreInputBuffering()
		fmt.Println()
		os.Exit(-2)
	}()
}

// Sign extend a value
func signExtend(x uint16, bitCount int) uint16 {
	if ((x >> (bitCount - 1)) & 1) != 0 {
		x |= (0xFFFF << bitCount)
	}
	return x
}

// Swap bytes in a 16-bit value
func swap16(x uint16) uint16 {
	return (x<<8) | (x >> 8)
}

// Update condition flags
func updateFlags(r uint16) {
	if reg[r] == 0 {
		reg[R_COND] = FL_ZRO
	} else if (reg[r] >> 15) != 0 { // a 1 in the left-most bit indicates negative
		reg[R_COND] = FL_NEG
	} else {
		reg[R_COND] = FL_POS
	}
}

// Read image file
func readImageFile(file *os.File) error {
	// Read the entire file
	fileInfo, err := file.Stat()
	if err != nil {
		return err
	}
	
	fileSize := fileInfo.Size()
	data := make([]byte, fileSize)
	if _, err := file.ReadAt(data, 0); err != nil {
		return err
	}
	
	// Get origin (first 2 bytes)
	if fileSize < 2 {
		return fmt.Errorf("file too small")
	}
	
	origin := binary.BigEndian.Uint16(data[:2])
	fmt.Printf("Origin: 0x%04X\n", origin)
	
	// Load program into memory
	for i := 2; i < len(data); i += 2 {
		if i+1 < len(data) {
			word := binary.BigEndian.Uint16(data[i:i+2])
			memory[origin+(uint16(i-2)/2)] = word
		}
	}
	
	return nil
}

// Read image
func readImage(imagePath string) error {
	file, err := os.Open(imagePath)
	if err != nil {
		return err
	}
	defer file.Close()

	return readImageFile(file)
}

// Write to memory
func memWrite(address, val uint16) {
	memory[address] = val
}

// Read from memory
func memRead(address uint16) uint16 {
	if address == MR_KBSR {
		if checkKey() {
			memory[MR_KBSR] = (1 << 15)
			var b [1]byte
			os.Stdin.Read(b[:])
			memory[MR_KBDR] = uint16(b[0])
		} else {
			memory[MR_KBSR] = 0
		}
	}
	return memory[address]
}

func main() {
	if len(os.Args) < 2 {
		fmt.Println("lc3 [image-file1] ...")
		os.Exit(2)
	}

	for _, imagePath := range os.Args[1:] {
		fmt.Printf("Loading image: %s\n", imagePath)
		if err := readImage(imagePath); err != nil {
			fmt.Printf("failed to load image: %s - %v\n", imagePath, err)
			os.Exit(1)
		}
		fmt.Println("Image loaded successfully")
	}

	handleInterrupt()
	disableInputBuffering()
	defer restoreInputBuffering()

	// Since exactly one condition flag should be set at any given time, set the Z flag
	reg[R_COND] = FL_ZRO

	// Set the PC to starting position
	reg[R_PC] = PC_START
	fmt.Printf("Starting execution at PC=0x%04X\n", reg[R_PC])

	running := true
	instructionCount := 0
	for running {
		// FETCH
		instr := memRead(reg[R_PC])
		reg[R_PC]++
		op := instr >> 12
		
		instructionCount++
		if instructionCount <= 10 {
			fmt.Printf("Executing instruction 0x%04X (op=%d) at PC=0x%04X\n", instr, op, reg[R_PC]-1)
		}

		switch op {
		case OP_ADD:
			// Destination register (DR)
			r0 := (instr >> 9) & 0x7
			// First operand (SR1)
			r1 := (instr >> 6) & 0x7
			// Whether we are in immediate mode
			immFlag := (instr >> 5) & 0x1

			if immFlag != 0 {
				imm5 := signExtend(instr&0x1F, 5)
				reg[r0] = reg[r1] + imm5
			} else {
				r2 := instr & 0x7
				reg[r0] = reg[r1] + reg[r2]
			}

			updateFlags(r0)

		case OP_AND:
			r0 := (instr >> 9) & 0x7
			r1 := (instr >> 6) & 0x7
			immFlag := (instr >> 5) & 0x1

			if immFlag != 0 {
				imm5 := signExtend(instr&0x1F, 5)
				reg[r0] = reg[r1] & imm5
			} else {
				r2 := instr & 0x7
				reg[r0] = reg[r1] & reg[r2]
			}
			updateFlags(r0)

		case OP_NOT:
			r0 := (instr >> 9) & 0x7
			r1 := (instr >> 6) & 0x7

			reg[r0] = ^reg[r1]
			updateFlags(r0)

		case OP_BR:
			pcOffset := signExtend(instr&0x1FF, 9)
			condFlag := (instr >> 9) & 0x7
			if (condFlag & reg[R_COND]) != 0 {
				reg[R_PC] += pcOffset
			}

		case OP_JMP:
			// Also handles RET
			r1 := (instr >> 6) & 0x7
			reg[R_PC] = reg[r1]

		case OP_JSR:
			longFlag := (instr >> 11) & 1
			reg[R_R7] = reg[R_PC]
			if longFlag != 0 {
				longPCOffset := signExtend(instr&0x7FF, 11)
				reg[R_PC] += longPCOffset // JSR
			} else {
				r1 := (instr >> 6) & 0x7
				reg[R_PC] = reg[r1] // JSRR
			}

		case OP_LD:
			r0 := (instr >> 9) & 0x7
			pcOffset := signExtend(instr&0x1FF, 9)
			reg[r0] = memRead(reg[R_PC] + pcOffset)
			updateFlags(r0)

		case OP_LDI:
			// Destination register (DR)
			r0 := (instr >> 9) & 0x7
			// PCoffset 9
			pcOffset := signExtend(instr&0x1FF, 9)
			// Add pcOffset to the current PC, look at that memory location to get the final address
			reg[r0] = memRead(memRead(reg[R_PC] + pcOffset))
			updateFlags(r0)

		case OP_LDR:
			r0 := (instr >> 9) & 0x7
			r1 := (instr >> 6) & 0x7
			offset := signExtend(instr&0x3F, 6)
			reg[r0] = memRead(reg[r1] + offset)
			updateFlags(r0)

		case OP_LEA:
			r0 := (instr >> 9) & 0x7
			pcOffset := signExtend(instr&0x1FF, 9)
			reg[r0] = reg[R_PC] + pcOffset
			updateFlags(r0)

		case OP_ST:
			r0 := (instr >> 9) & 0x7
			pcOffset := signExtend(instr&0x1FF, 9)
			memWrite(reg[R_PC]+pcOffset, reg[r0])

		case OP_STI:
			r0 := (instr >> 9) & 0x7
			pcOffset := signExtend(instr&0x1FF, 9)
			memWrite(memRead(reg[R_PC]+pcOffset), reg[r0])

		case OP_STR:
			r0 := (instr >> 9) & 0x7
			r1 := (instr >> 6) & 0x7
			offset := signExtend(instr&0x3F, 6)
			memWrite(reg[r1]+offset, reg[r0])

		case OP_TRAP:
			reg[R_R7] = reg[R_PC]

			switch instr & 0xFF {
			case TRAP_GETC:
				// Read a single ASCII char
				var b [1]byte
				os.Stdin.Read(b[:])
				reg[R_R0] = uint16(b[0])
				updateFlags(R_R0)

			case TRAP_OUT:
				fmt.Printf("%c", byte(reg[R_R0]))
				os.Stdout.Sync()

			case TRAP_PUTS:
				// One char per word
				addr := reg[R_R0]
				for memory[addr] != 0 {
					fmt.Printf("%c", byte(memory[addr]))
					addr++
				}
				os.Stdout.Sync()

			case TRAP_IN:
				fmt.Print("Enter a character: ")
				var b [1]byte
				os.Stdin.Read(b[:])
				fmt.Printf("%c", b[0])
				os.Stdout.Sync()
				reg[R_R0] = uint16(b[0])
				updateFlags(R_R0)

			case TRAP_PUTSP:
				// One char per byte (two bytes per word)
				// Here we need to swap back to big endian format
				addr := reg[R_R0]
				for memory[addr] != 0 {
					char1 := byte(memory[addr] & 0xFF)
					fmt.Printf("%c", char1)
					char2 := byte(memory[addr] >> 8)
					if char2 != 0 {
						fmt.Printf("%c", char2)
					}
					addr++
				}
				os.Stdout.Sync()

			case TRAP_HALT:
				fmt.Println("HALT")
				os.Stdout.Sync()
				running = false
			}

		case OP_RES:
			fmt.Println("Reserved operation")
			running = false
		case OP_RTI:
			fmt.Println("RTI operation")
			running = false
		default:
			fmt.Printf("Invalid operation: %d\n", op)
			running = false
		}
	}
}
