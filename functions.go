package main

import (
	"math"
	"math/rand"
	"time"
)

//place your non-drawing functions here.

// Calculate the Euclidean distance between two points in 2D space
func Distance(b1_pos, b2_pos OrderedPair) float64 {

	delta_x := b1_pos.x - b2_pos.x
	delta_y := b1_pos.y - b2_pos.y

	return math.Sqrt(delta_x * delta_x + delta_y * delta_y)
}

//Return a slice of Sky objects representing the time evolution of the boid system
func SimulateBoids(initial_sky Sky, num_gens int, time_step float64) []Sky {
	time_steps := make([]Sky, num_gens + 1)
	time_steps[0] = initial_sky

	for i := 1; i < (num_gens + 1); i++ {
		time_steps[i] = UpdateSky(time_steps[i-1], time_step)
	}

	return time_steps
}

// UpdateSky takes in the current sky and time step, and returns the updated sky after one time step
func UpdateSky(current_sky Sky, time_step float64) Sky {
	new_sky := CopySky(current_sky)

	max_speed := current_sky.maxBoidSpeed
	sky_width := current_sky.width


	for i, b := range new_sky.boids {
		old_acceleration, old_velocity := b.acceleration, b.velocity

		new_sky.boids[i].acceleration = UpdateAcceleration(current_sky, i)
		new_sky.boids[i].velocity = UpdateVelocity(new_sky.boids[i], old_acceleration, max_speed, time_step)
		new_sky.boids[i].position = UpdatePosition(new_sky.boids[i], old_acceleration, old_velocity, sky_width, time_step)
	}

	return new_sky
}

// Update the acceleration of boid i (index) in current_sky
func UpdateAcceleration(current_sky Sky, i int) OrderedPair {
	var accel OrderedPair

	b := current_sky.boids[i]

	force := ComputeNetForce(current_sky, b)
	accel.x = force.x / 1.0
	accel.y = force.y / 1.0

	return accel
}

// Update the velocity of boid b given its old acceleration and time step
func UpdateVelocity(b Boid, old_acceleration OrderedPair, MaxBoidSpeed, time_step float64) OrderedPair {
	var velo OrderedPair
	max_speed := MaxBoidSpeed
	
	velo.x = b.velocity.x + 0.5 * (b.acceleration.x + old_acceleration.x) * time_step
	velo.y = b.velocity.y + 0.5 * (b.acceleration.y + old_acceleration.y) * time_step

	//check if velocity exceeds maxBoidSpeed
	velo_value := math.Sqrt(velo.x * velo.x + velo.y * velo.y)

	// if it does, scale it down to maxBoidSpeed
	if velo_value > max_speed {
		velo.x = velo.x * (max_speed / velo_value)
		velo.y = velo.y * (max_speed / velo_value)
	}

	return velo
}

// Update the position of boid b given its old acceleration, old velocity, sky width and time step
func UpdatePosition(b Boid, old_acceleration, old_velocity OrderedPair, SkyWidth, time_step float64) OrderedPair {
	var pos OrderedPair
	
	pos.x = b.position.x + old_velocity.x * time_step + 0.5 * old_acceleration.x * time_step * time_step
	pos.y = b.position.y + old_velocity.y * time_step + 0.5 * old_acceleration.y * time_step * time_step
	
	// wrap around the sky if the boid goes out of bounds
	// it may not work if directly add/ substract SkyWidth to pos.x or pos.y
	if pos.x == SkyWidth {
		pos.x = pos.x
	} else {
		pos.x = math.Mod(pos.x + SkyWidth, SkyWidth)
	}
	
	if pos.y == SkyWidth {
		pos.y = pos.y
	} else {
		pos.y = math.Mod(pos.y + SkyWidth, SkyWidth)
	}
	

	return pos
}

// Compute the net force on boid b from all other boids in current_sky
func ComputeNetForce(current_sky Sky, b Boid) OrderedPair {
	var sep_force, align_force, coh_force OrderedPair
	var force OrderedPair
	neighbor_count := 0

	for i := range current_sky.boids {
		if current_sky.boids[i] != b {
			S := current_sky.separationFactor
			A := current_sky.alignmentFactor
			C := current_sky.cohesionFactor
			R := current_sky.proximity
			
			d := Distance(b.position, current_sky.boids[i].position)

			// birds in same position: force = 0
			if d == 0 {
				continue
			}

			// check whether two birds are within the proximity distance
			if d < R {
				neighbor_count++ // neighbor_count whithin proximity distance, used to average the forces later
				s_force := ComputeSeparationForce(b, current_sky.boids[i], S, d)
				a_force := ComputeAlignmentForce(b, current_sky.boids[i], A, d)
				c_force := ComputeCohesionForce(b, current_sky.boids[i], C, d)

				sep_force.x += s_force.x
				sep_force.y += s_force.y
				align_force.x += a_force.x
				align_force.y += a_force.y
				coh_force.x += c_force.x
				coh_force.y += c_force.y
			}
		}
	}

	// average the forces
	if neighbor_count > 0 {
		sep_force.x /= float64(neighbor_count)
		sep_force.y /= float64(neighbor_count)
		align_force.x /= float64(neighbor_count)
		align_force.y /= float64(neighbor_count)
		coh_force.x /= float64(neighbor_count)
		coh_force.y /= float64(neighbor_count)
	}			
	
	force.x += (sep_force.x + align_force.x + coh_force.x)
	force.y += (sep_force.y + align_force.y + coh_force.y)

	return force
}

// Compute the separation force exerted on boid b1 by boid b2
func ComputeSeparationForce(b1, b2 Boid, S, distance float64) OrderedPair {
	var s_force OrderedPair

	s_force.x = S * (b1.position.x - b2.position.x) / (distance * distance)
	s_force.y = S * (b1.position.y - b2.position.y) / (distance * distance)

	return s_force
}

// Compute the alignment force exerted on boid b1 by boid b2
func ComputeAlignmentForce(b1, b2 Boid, A, distance float64) OrderedPair {
	var a_force OrderedPair

	a_force.x = A * (b2.velocity.x) / distance
	a_force.y = A * (b2.velocity.y) / distance

	return a_force
}

// Compute the cohesion force exerted on boid b1 by boid b2
func ComputeCohesionForce(b1, b2 Boid, C, distance float64) OrderedPair {
	var c_force OrderedPair

	c_force.x = C * (b2.position.x - b1.position.x) / distance
	c_force.y = C * (b2.position.y - b1.position.y) / distance

	return c_force
}


// Mannual deep copy of sky and boid
func CopySky(current_sky Sky) Sky {
	var new_sky Sky

	new_sky.width = current_sky.width
	new_sky.proximity = current_sky.proximity
	new_sky.separationFactor = current_sky.separationFactor
	new_sky.alignmentFactor = current_sky.alignmentFactor
	new_sky.cohesionFactor = current_sky.cohesionFactor
	new_sky.maxBoidSpeed = current_sky.maxBoidSpeed
	new_sky.boids = make([]Boid, len(current_sky.boids))
	
	for i := range current_sky.boids {
		new_sky.boids[i] = CopyBoid(current_sky.boids[i])
	}

	return new_sky
}

func CopyBoid(b Boid) Boid {
	var new_boid Boid

	new_boid.position.x = b.position.x 
	new_boid.position.y = b.position.y

	new_boid.velocity.x = b.velocity.x
	new_boid.velocity.y = b.velocity.y
	
	new_boid.acceleration.x = b.acceleration.x
	new_boid.acceleration.y = b.acceleration.y 

	return new_boid
}

// Generate random sky with num_boids boids from input parameters
func GenerateRandomSky(num_boids int, 
	sky_width, initial_speed, max_boid_speed, proximity, 
	separation_factor, alignment_factor, cohesion_factor float64) Sky {
		var initial_sky Sky
		
		initial_sky.width = sky_width
		initial_sky.proximity = proximity
		initial_sky.separationFactor = separation_factor
		initial_sky.alignmentFactor = alignment_factor
		initial_sky.cohesionFactor = cohesion_factor
		initial_sky.maxBoidSpeed = max_boid_speed
		initial_sky.boids = make([]Boid, num_boids)

		rand.Seed(time.Now().UnixNano())

		// for_, b := range ...: get copy of b thus can not change element in the slice, so deep copy is needed
		for i := range initial_sky.boids {
			initial_sky.boids[i].position.x = rand.Float64() * sky_width
			initial_sky.boids[i].position.y = rand.Float64() * sky_width

			theta := rand.Float64() * 2.0 * math.Pi // random angle in [0, 2pi)
			initial_sky.boids[i].velocity.x = initial_speed * math.Cos(theta)
			initial_sky.boids[i].velocity.y = initial_speed * math.Sin(theta)

			initial_sky.boids[i].acceleration.x = 0.0
			initial_sky.boids[i].acceleration.y = 0.0
		}

		return initial_sky
	}