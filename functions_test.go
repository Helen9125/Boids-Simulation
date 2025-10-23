//write testing code for each function in functions.go
//using the tests in the Tests file

package main

import (
	"bufio"
	"io/fs"
	"os"
	"math"
	"strconv"
	"strings"
	"testing"
	// "fmt"
)

// DistanceTest is a struct that holds the information for a test of the Distance function
type DistanceTest struct {
	b1_pos OrderedPair
	b2_pos OrderedPair
	result float64
}

// ComputeTest is a struct that holds the information for a test of four force-related functions
type ComputeTest struct {
	b1_pos OrderedPair
	b2_pos OrderedPair
	b2_vel OrderedPair
	proximity float64
	separation_factor float64
	align_factor float64
	cohesion_factor float64
	result OrderedPair
}

// TestDistance tests the Distance function
func TestDistance(t *testing.T) {
	//read in all tests from the Tests/Distance directory and run them
	tests := ReadDistanceTests("Tests/Distance/")
	for _, test := range tests {
		//run the test
		result := Distance(test.b1_pos, test.b2_pos)

		epsilon := 1e-2 // tolerance for floating-point comparison

		if math.Abs(result - test.result) > epsilon {
    		t.Errorf("TestDistance(b1_pos: %v, b2_pos: %v) = %v, want %v",
        		test.b1_pos, test.b2_pos, result, test.result)
		}
	}
}



// ReadDirectory reads in a directory and returns a slice of fs.DirEntry objects containing file info for the directory
func ReadDirectory(dir string) []fs.DirEntry {
	//read in all files in the given directory
	files, err := os.ReadDir(dir)
	if err != nil {
		panic(err)
	}
	return files
}

// ReadDistanceTests takes as input a directory and returns a slice of DistanceTest objects
func ReadDistanceTests(directory string) []DistanceTest {

	//read in all tests from the directory and run them
	input_files := ReadDirectory(directory + "/input")
	num_files := len(input_files)

	tests := make([]DistanceTest, num_files)
	for i, input_file := range input_files {
		//read in the tests
		b1, b2 := ReadDistanceSample(directory + "input/" + input_file.Name())
		tests[i].b1_pos = b1
		tests[i].b2_pos = b2
	}

	//now, read output files
	output_files := ReadDirectory(directory + "/output")
	if len(output_files) != num_files {
		panic("Error: number of input and output files do not match!")
	}

	for i, output_file := range output_files {
		//read in the test's result
		tests[i].result = ReadFloatFromFile(directory + "output/" + output_file.Name())
	}

	return tests
}

// ReadDistanceSample reads in position OrderedPair from a file
func ReadDistanceSample(file string) (OrderedPair, OrderedPair) {
	//open the file
	f, err := os.Open(file)
	Check(err)
	defer f.Close()

	//create a new scanner
	scanner := bufio.NewScanner(f)

	//while the scanner still has lines to read,
	//read in the next line
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		//skip comment lines
		if strings.HasPrefix(line, "#") || line == "" {
			continue
		}

		parts := strings.Fields(line) // split by any whitespace
		if len(parts) != 4 {
			panic("Error: each line must contain exactly four values")
		}
		
		x1, err := strconv.ParseFloat(parts[0], 64)
		Check(err)
		y1, err := strconv.ParseFloat(parts[1], 64)
		Check(err)
		x2, err := strconv.ParseFloat(parts[2], 64)
		Check(err)
		y2, err := strconv.ParseFloat(parts[3], 64)
		Check(err)

		return OrderedPair{x: x1, y: y1}, OrderedPair{x: x2, y: y2}

	}
	panic("Error: no valid data line found")
}

// ReadFloatFromFile reads in a single float from a file
func ReadFloatFromFile(file string) float64 {
	//open the file
	f, err := os.Open(file)
	Check(err)
	defer f.Close()

	//create a new scanner
	scanner := bufio.NewScanner(f)

	//read in the line
	scanner.Scan()
	line := scanner.Text()

	//convert the line to an int using strconv
	value, err := strconv.ParseFloat(line, 64)
	Check(err)

	return value
}


//
// Test ComputeSeparationForce
// In function.go, we check if two boids are within proximity distance in ComputeNetForce
// So we call Compute Force functions only when two boids are within proximity distance

// So, here, we only need to test ComputeSeparationForce function for every pair of boids without considering proximity distance
// No pairs with same position (distance = 0) in the test cases
func TestComputeSeparationForce(t *testing.T) {
	tests := ReadComputeForceTests("Tests/ComputeSeparationForce/")

	for _, test := range tests {

		distance := Distance(test.b1_pos, test.b2_pos)

		// function takes in two boids, so create two boids
		b1 := Boid{position: test.b1_pos}
		b2 := Boid{position: test.b2_pos, velocity: test.b2_vel}

		//run the test
		result := ComputeSeparationForce(b1, b2, test.separation_factor, distance)

		epsilon := 1e-2 // tolerance for floating-point comparison

		if math.Abs(result.x - test.result.x) > epsilon || math.Abs(result.y - test.result.y) > epsilon {
    		t.Errorf("ComputeSeparationForce(b1_pos: %v, b2_pos: %v, S_factor: %v) = %v, want %v",
        		test.b1_pos, test.b2_pos, test.separation_factor, result, test.result)
		}
	}
}

func TestComputeAlignmentForce(t *testing.T) {
	tests := ReadComputeForceTests("Tests/ComputeAlignmentForce/")

	for _, test := range tests {

		distance := Distance(test.b1_pos, test.b2_pos)

		// function takes in two boids, so create two boids
		b1 := Boid{position: test.b1_pos}
		b2 := Boid{position: test.b2_pos, velocity: test.b2_vel}

		//run the test
		result := ComputeAlignmentForce(b1, b2, test.align_factor, distance)

		epsilon := 1e-2 // tolerance for floating-point comparison

		if math.Abs(result.x - test.result.x) > epsilon || math.Abs(result.y - test.result.y) > epsilon {
    		t.Errorf("ComputeAlignmentForce(b1_pos: %v, b2_pos: %v, b2_vel: %v, A_factor: %v) = %v, want %v",
        		test.b1_pos, test.b2_pos, test.b2_vel, test.align_factor, result, test.result)
		}
	}
}

func TestComputeCohesionForce(t *testing.T) {
	tests := ReadComputeForceTests("Tests/ComputeCohesionForce/")

	for _, test := range tests {

		distance := Distance(test.b1_pos, test.b2_pos)

		// function takes in two boids, so create two boids
		b1 := Boid{position: test.b1_pos}
		b2 := Boid{position: test.b2_pos, velocity: test.b2_vel}

		//run the test
		result := ComputeCohesionForce(b1, b2, test.cohesion_factor, distance)

		epsilon := 1e-2 // tolerance for floating-point comparison

		if math.Abs(result.x - test.result.x) > epsilon || math.Abs(result.y - test.result.y) > epsilon {
    		t.Errorf("ComputeCohesionForce(b1_pos: %v, b2_pos: %v, C_factor: %v) = %v, want %v",
        		test.b1_pos, test.b2_pos, test.cohesion_factor, result, test.result)
		}
	}
}


// functions to read in tests for Compute Force functions
func ReadComputeForceTests(directory string) []ComputeTest {
	input_files := ReadDirectory(directory + "/input")
	num_files := len(input_files)

	tests := make([]ComputeTest, num_files)
	for i, input_file := range(input_files) {
		b1_pos, b2_pos, b2_vel, proximity, S_factor, A_factor, C_factor := ReadComputeForceSample(directory + "input/" + input_file.Name())
		tests[i].b1_pos = b1_pos
		tests[i].b2_pos = b2_pos
		tests[i].b2_vel = b2_vel
		tests[i].proximity = proximity
		tests[i].separation_factor = S_factor
		tests[i].align_factor = A_factor
		tests[i].cohesion_factor = C_factor
	}

	//read output files
	output_files := ReadDirectory(directory + "/output")
	if len(output_files) != num_files {
		panic("Error: number of input and output files do not match!")
	}

	for i, output_file := range(output_files) {
		tests[i].result = ReadOrderedPairFromFile(directory + "output/" + output_file.Name())
	}

	return tests
}

// ReadComputeForceSample reads in testing data and expected output for Compute Force functions
func ReadComputeForceSample(directory string) (OrderedPair, OrderedPair, OrderedPair, float64, float64, float64, float64) {
	// open a file
	f, err := os.Open(directory)
	Check(err)
	defer f.Close()

	// create a new scanner & read in line
	scanner := bufio.NewScanner(f)

	scanner.Scan()

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		//skip comment lines
		if strings.HasPrefix(line, "#") || line == "" {
			continue
		}

		fields := strings.Fields(line)
		if len(fields) != 10 {
			panic("Error: each line in Compute force input must contain exactly seven values")
		}

		b1_pos_x, err := strconv.ParseFloat(fields[0], 64)
		Check(err)
		b1_pos_y, err := strconv.ParseFloat(fields[1], 64)
		Check(err)
		b2_pos_x, err := strconv.ParseFloat(fields[2], 64)
		Check(err)
		b2_pos_y, err := strconv.ParseFloat(fields[3], 64)
		Check(err)
		b2_vel_x, err := strconv.ParseFloat(fields[4], 64)
		Check(err)
		b2_vel_y, err := strconv.ParseFloat(fields[5], 64)
		Check(err)
		proximity, err := strconv.ParseFloat(fields[6], 64)
		Check(err)
		S_factor, err := strconv.ParseFloat(fields[7], 64)
		Check(err)
		A_factor, err := strconv.ParseFloat(fields[8], 64)
		Check(err)
		C_factor, err := strconv.ParseFloat(fields[9], 64)
		Check(err)

		b1_pos := OrderedPair{x: b1_pos_x, y: b1_pos_y}
		b2_pos := OrderedPair{x: b2_pos_x, y: b2_pos_y}
		b2_vel := OrderedPair{x: b2_vel_x, y: b2_vel_y}

		return b1_pos, b2_pos, b2_vel, proximity, S_factor, A_factor, C_factor 
	}
	panic("Error: no valid data line found")
}

// ReadOrderedPairFromFile reads in a single float from a file
func ReadOrderedPairFromFile(file string) OrderedPair {
	//open the file
	f, err := os.Open(file)
	Check(err)
	defer f.Close()

	//create a new scanner
	scanner := bufio.NewScanner(f)

	//read in the line
	scanner.Scan()
	line := scanner.Text()

	fields := strings.Fields(line)
	if len(fields) != 2 {
		panic("Error: each line must contain exactly two values")
	}

	x, err := strconv.ParseFloat(fields[0], 64)
	Check(err)
	y, err := strconv.ParseFloat(fields[1], 64)
	Check(err)

	return OrderedPair{x: x, y: y}
}
