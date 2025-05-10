package configurator

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"zpicier/core/logger"

	"gopkg.in/yaml.v3"
)

type Configurator struct {
	config map[string]string
	logger *logger.Logger
}

var (
	instance     *Configurator
	once         sync.Once
	configPaths  []string
	configMux    sync.Mutex
	initializing bool
)

// AddConfigPath queues a YAML path for loading during Init
func AddConfigPath(path string) {
	configMux.Lock()
	defer configMux.Unlock()
	if initializing {
		instance.logger.LogInPlaceWarning("Config path added after Init(): %s (ignored)\n", path)
		return
	}
	configPaths = append(configPaths, path)
}

// setBasePath sets the base path for config files
func setBasePath(basePath string) {
	configMux.Lock()
	defer configMux.Unlock()
	if initializing {
		instance.logger.LogInPlaceWarning("Base path set after Init(): %s (ignored)\n", basePath)
		return
	}
	for i, path := range configPaths {
		configPaths[i] = basePath+path
	}

}
func getConfigPath() (string, error) {
	exePath, err := os.Executable()
	if err != nil {
		return "", err
	}
	dir := filepath.Dir(exePath)

	for {
		configPath := filepath.Join(dir, "config")
		info, err := os.Stat(configPath)
		if err == nil && info.IsDir() {
			// Found directory containing "config"
			return dir, nil
		}

		parent := filepath.Dir(dir)
		if parent == dir {
			// Reached filesystem root
			return "", os.ErrNotExist
		}
		dir = parent
	}
}

// Init loads all queued YAML files into a merged config
func Init() error {
	var err error
	rootDir, err := getConfigPath()
	if err != nil {
		return instance.logger.LogInPlaceError("could not find project root (with /config): %v", err)
	}
	setBasePath(rootDir+"/")
	once.Do(func() {
		initializing = true
		cfg := &Configurator{logger: logger.NewLogger()}
		cfg.config, err = loadYAMLFiles(configPaths)
		instance = cfg
	})
	return err
}

// loadYAMLFiles loads and flattens all config files into a string-string map
func loadYAMLFiles(paths []string) (map[string]string, error) {
	merged := make(map[string]string)

	for _, path := range paths {
		data, err := os.ReadFile(path)
		if err != nil {
			return nil, instance.logger.LogInPlaceError("failed to read config file %s: %v", path, err)
		}

		raw := make(map[string]interface{})
		if err := yaml.Unmarshal(data, &raw); err != nil {
			return nil, instance.logger.LogInPlaceError("failed to parse YAML %s: %v", path, err)
		}

		for k, v := range raw {
			merged[strings.ToUpper(k)] = fmt.Sprintf("%v", v)
		}
	}

	return merged, nil
}

// checks if exists returns the value for a key (case-insensitive, stored upper)
func checkFetch(key string) (string, bool) {
	if instance == nil {
		return "", false
	}
	val, ok := instance.config[strings.ToUpper(key)]
	return val, ok
}
func Get(key string) string {
	val, ok := checkFetch(key)
	if !ok {
		panic("config key not found: " + key)
	}
	return val
}

// GetAll returns the merged flat map
func GetAll() map[string]string {
	if instance == nil {
		return nil
	}
	return instance.config
}
