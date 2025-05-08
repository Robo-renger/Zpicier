package env_params

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
)

type EnvParams struct {
	data map[string]string
}

var (
	instance *EnvParams
	once     sync.Once
)

// Init loads .env from the project root (found via upward traversal)
func Init() error {
	var err error
	once.Do(func() {
		root, errRoot := findProjectRoot()
		if errRoot != nil {
			err = errRoot
			return
		}

		envPath := filepath.Join(root, ".env")
		data, errFile := readEnvFile(envPath)
		if errFile != nil {
			err = errFile
			return
		}

		instance = &EnvParams{data: data}
	})
	return err
}

// checks if exists returns the value for a key
func checkFetch(key string) (string, bool) {
	if instance == nil {
		return "", false
	}
	val, ok := instance.data[strings.ToUpper(key)]
	return val, ok
}

// MustGet returns the value or panics
func Get(key string) string {
	val, ok := checkFetch(key)
	if !ok {
		panic("ENV key not found: " + key)
	}
	return val
}

// findProjectRoot traverses upward until it finds .env
func findProjectRoot() (string, error) {
	exePath, err := os.Executable()
	if err != nil {
		return "", err
	}
	dir := filepath.Dir(exePath)

	for {
		envPath := filepath.Join(dir, ".env")
		if _, err := os.Stat(envPath); err == nil {
			return dir, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			break
		}
		dir = parent
	}
	return "", fmt.Errorf(".env not found near executable")
}


// readEnvFile reads KEY=VAL lines from a file
func readEnvFile(path string) (map[string]string, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	data := make(map[string]string)
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		line = strings.TrimSpace(line)

		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}

		if kv := strings.SplitN(line, "=", 2); len(kv) == 2 {
			k := strings.ToUpper(strings.TrimSpace(kv[0]))
			v := strings.TrimSpace(kv[1])
			data[k] = v
		}
	}
	return data, scanner.Err()
}
