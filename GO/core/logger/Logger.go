package logger

import (
	"errors"
	"fmt"
	"os"
)

type Logger struct {
	file *os.File
}

func NewLogger() *Logger {
	return &Logger{}
}

// ANSI color codes
const (
	ColorReset  = "\033[0m"
	ColorRed    = "\033[31m"
	ColorGreen  = "\033[32m"
	ColorYellow = "\033[33m"
	ColorCyan   = "\033[36m"
)

func (l *Logger) LogInPlaceInfo(format string, a ...interface{}) {
	msg := fmt.Sprintf(format, a...)
	fmt.Printf("\r%s[INFO] %s%s\n", ColorCyan, msg, ColorReset)
}

func (l *Logger) LogInPlaceError(format string, a ...interface{}) error {
	msg := fmt.Sprintf(format, a...)
	fmt.Printf("\r%s[ERROR] %s%s\n", ColorRed, msg, ColorReset)
	return errors.New(msg)
}

func (l *Logger) LogInPlaceSuccess(format string, a ...interface{}) {
	msg := fmt.Sprintf(format, a...)
	fmt.Printf("\r%s[SUCCESS] %s%s\n", ColorGreen, msg, ColorReset)
}

func (l *Logger) LogInPlaceWarning(format string, a ...interface{}) {
	msg := fmt.Sprintf(format, a...)
	fmt.Printf("\r%s[WARNING] %s%s\n", ColorYellow, msg, ColorReset)
}
