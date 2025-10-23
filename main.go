package main

import (
	"fmt"
	"os"
	"strconv"
	"gifhelper"
)

func main() {
	fmt.Println("Hacking boids!")

	// Process your command-line arguments here
	if len(os.Args) != 13 {
		panic("Error: incorrect number of command line arguments.")
	}

	// take CLAs
	// ./boids numBoids skyWidth initialSpeed maxBoidSpeed numGens proximity separationFactor
	// alignmentFactor cohesionFactor timeStep canvasWidth imageFrequency

	// than initial_sky will be generated with these parameters
	num_boids, err_1 := strconv.Atoi(os.Args[1])
	Check(err_1)

	sky_width , err_2 := strconv.ParseFloat(os.Args[2], 64)
	Check(err_2)

	initial_speed , err_3 := strconv.ParseFloat(os.Args[3], 64)
	Check(err_3)

	max_boid_speed , err_4 := strconv.ParseFloat(os.Args[4], 64)
	Check(err_4)

	num_gens, err_5 := strconv.Atoi(os.Args[5])
	Check(err_5)

	proximity, err_6 := strconv.ParseFloat(os.Args[6], 64)
	Check(err_6)

	separation_factor, err_7 := strconv.ParseFloat(os.Args[7], 64)
	Check(err_7)

	alignment_factor, err_8 := strconv.ParseFloat(os.Args[8], 64)
	Check(err_8)

	cohesion_factor, err_9 := strconv.ParseFloat(os.Args[9], 64)
	Check(err_9)
	
	time_step, err_10 := strconv.ParseFloat(os.Args[10], 64)
	Check(err_10)

	canvas_width, err_11 := strconv.Atoi(os.Args[11])
	Check(err_11)

	image_frequency, err_12 := strconv.Atoi(os.Args[12])
	Check(err_12)

	if image_frequency <= 0 {
		panic("Error: nonpositive number as drawing_frequency")
	}

	output_file := "output/" + "test_boids"

	fmt.Println("Command line arguements read")

	fmt.Println("Simulating boids")

	// generate initial sky
	initial_sky := GenerateRandomSky(num_boids, sky_width, initial_speed, max_boid_speed, proximity, separation_factor, alignment_factor, cohesion_factor)
	fmt.Println("Initial sky generated")

	// Call simulation function
	time_points := SimulateBoids(initial_sky, num_gens, time_step)
	fmt.Println("Simulation run")

	// Defining configuration settings for animation.
	config := Config{
		CanvasWidth:     canvas_width,
		BoidSize:        5.0, // Set the boid size
		BoidColor:       Color{R: 255, G: 255, B: 255, A: 255},
		BackgroundColor: Color{R: 173, G: 216, B: 230}, // Light blue background
	}

	// Call AnimateSystem using the config parameter
	fmt.Println("Drawing sky")
	images := AnimateSystem(time_points, config, image_frequency)
	fmt.Println("Image drawn")

	// Then, render an animated GIF.
	fmt.Println("Making GIF")
	gifhelper.ImagesToGIF(images, output_file)
	fmt.Println("GIF drawn!")
}

func Check(err error) {
	if err != nil {
		panic(err)
	}
}
