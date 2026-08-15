package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"
)

// Cleanup functions for signal-path exits: os.Exit skips defers, so anything
// that must run on SIGINT/SIGTERM (restoring the terminal from liner's raw
// mode, saving history) registers here. Run in LIFO order.
var (
	cleanupMu    sync.Mutex
	cleanupFuncs []func()
)

func registerCleanup(f func()) {
	cleanupMu.Lock()
	defer cleanupMu.Unlock()
	cleanupFuncs = append(cleanupFuncs, f)
}

func runCleanups() {
	cleanupMu.Lock()
	fns := cleanupFuncs
	cleanupFuncs = nil
	cleanupMu.Unlock()
	for i := len(fns) - 1; i >= 0; i-- {
		fns[i]()
	}
}

func main() {
	os.Exit(run())
}

func run() int {
	// ---- 1. global flags ----
	cfgFile := flag.String("config", "", "Path to JSON config")
	verbose := flag.Bool("v", false, "Verbose logging")
	jsonFlag := flag.Bool("json", false, "JSON output for status/position commands")
	flag.Parse()
	jsonOutput = *jsonFlag

	// ---- 2. config & logger ----
	cfg, err := LoadConfig(*cfgFile)
	if err != nil {
		fmt.Fprintf(os.Stderr, "config error: %v\n", err)
		return 1
	}
	logger := log.New(os.Stdout, "[XU81] ", log.LstdFlags)
	if *verbose {
		logger.SetFlags(log.LstdFlags | log.Lshortfile)
	}

	// ---- 3. camera ----
	cam := NewXU81Camera(cfg, logger)
	defer cam.Close()

	// Safety: a killed process must not leave the head panning — the
	// pan/tilt mechanism is rated for 300K cycles. The goroutine only ever
	// wakes on a delivered signal, so normal exits never trigger the stop.
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() {
		s := <-sig
		logger.Printf("shutting down (%v): stopping camera", s)
		cam.BestEffortStop()
		cam.Close()
		runCleanups()
		os.Exit(130)
	}()

	// ---- 4. CLI args or interactive ----
	if flag.NArg() > 0 {
		if err := runCommand(cam, flag.Arg(0), flag.Args()[1:]); err != nil {
			fmt.Fprintf(os.Stderr, "error: %v\n", err)
			return 1
		}
		return 0
	}

	return runInteractive(cam)
}
