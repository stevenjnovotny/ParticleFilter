/**
 * particle_filter.cpp
 *
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * Add random Gaussian noise to each particle.
   */
  num_particles = 50;  // Set the number of particles

  std::default_random_engine gen;

  // create normal distribution for x, y, theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
    weights.push_back(1.0);
  }

  is_initialized = true;
  std::cout << "particle filter is initialized with " << particles.size() << " particles" << std::endl;
  averageValues();
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * Add measurements to each particle and add random Gaussian noise.
   */
  
  std::default_random_engine gen;

  // x = x0 + v/theta_dot * (sin(theta0 + theta_dot * delta_t) - sin(theta0))
  // y = y0 + v/theta_dot * (cos(theta0) - cos(theta0 + theta_dot * delta_t))
  // theta = theta0 + theta_dot * delta_t
  // std::cout << velocity << " " << yaw_rate << std::endl;
  // for (int i = 0; i < num_particles; i++) {
  for (auto &p : particles) {  
  	double x0 = p.x;
  	double y0 = p.y;
  	double theta_0 = p.theta ;  
    
    double x_i;
    double y_i;
    double theta_i;
    
//     x_i = x0 + (velocity/yaw_rate) * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
//    	y_i = y0 + (velocity/yaw_rate) * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));    
//   	theta_i = theta_0 + yaw_rate * delta_t;
    
    if (yaw_rate != 0) {
    	x_i = x0 + (velocity/yaw_rate) * (sin(theta_0 + yaw_rate * delta_t) - sin(theta_0));
    	y_i = y0 + (velocity/yaw_rate) * (cos(theta_0) - cos(theta_0 + yaw_rate * delta_t));    
  		theta_i = theta_0 + yaw_rate * delta_t;
    } else {
        x_i = x0 + velocity * delta_t * cos(theta_0);
    	y_i = y0 + velocity * delta_t * sin(theta_0);    
  		theta_i = theta_0;
    }
    

  	// create normal distribution for x, y, theta
  	normal_distribution<double> noise_x(x_i, std_pos[0]);
  	normal_distribution<double> noise_y(y_i, std_pos[1]);
  	normal_distribution<double> noise_theta(theta_i, std_pos[2]);
    p.x = noise_x(gen);
  	p.y = noise_y(gen);
  	p.theta = noise_theta(gen);
    
  }
  
  // averageValues();
}

void ParticleFilter::averageValues() {
  
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (int i = 0; i < num_particles; i++) {
    sum_x += particles[i].x;
    sum_y += particles[i].y;
  }
  
  std::cout << "x_average: " << sum_x/num_particles << "  y_average: " << sum_y/num_particles << std::endl;
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   *   helper during the updateWeights phase.
   */
  
  for (auto &obs : observations) {
    // std::cout << obs.id << "  " << obs.x << "  " << obs.y << std::endl; 
    double min_d = 1.0e10;
    LandmarkObs best_lm;
    for (auto lm : predicted) {
	  double d = dist(lm.x, lm.y, obs.x, obs.y);     
      if (d < min_d) {
        min_d = d;
        best_lm = lm;
      }      
    }
    // std::cout << " -> observation is likely # " << best_lm.id << " at " << best_lm.x << " " << best_lm.y << std::endl;
    obs.id = best_lm.id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   */
  
  double w_norm = 0.0;
  for (auto &p : particles) {
     
    vector<LandmarkObs> predicted;  
    // Particle p = particles[0];

    for (std::size_t i = 0; i < map_landmarks.landmark_list.size(); i++) {
      Map::single_landmark_s ml = map_landmarks.landmark_list[i];
      //std::cout << ml.id_i << "  " << ml.x_f << "  " << ml.y_f << std::endl;

      double d = dist(p.x, p.y, ml.x_f, ml.y_f);
      if (d < sensor_range) {
        LandmarkObs lm;
        lm.id = ml.id_i;
        lm.x = ml.x_f;
        lm.y = ml.y_f;
        predicted.push_back(lm);  
      }
    } 

    /*
     transformation
      xm    	cos	 -sin	xp		xobs
      ym	=	sin  cos	yp * 	yobs
      1		0	 0		1 		1
    */

    // transform observations into map coordinates
    
    vector<LandmarkObs> obs; 
    for (std::size_t i = 0; i < observations.size(); i++) {

      LandmarkObs obs_i = observations[i];

      // std::cout << obs_i.x << " " << obs_i.y << std::endl;

      double pxt = cos(p.theta) * obs_i.x - sin(p.theta) * obs_i.y + p.x;
      double pyt = sin(p.theta) * obs_i.x + cos(p.theta) * obs_i.y + p.y;

      // std::cout << pxt << " " << pyt << std::endl;

      LandmarkObs obs_new;
      obs_new.x = pxt;
      obs_new.y = pyt;
      obs_new.id = observations[i].id;
      obs.push_back(obs_new);    
    }


   	// associate landmarks with observations
    dataAssociation(predicted, obs);
    
    // add associations to particle
    vector<int> a;
    vector<double> sx;
    vector<double> sy;
    for (auto o_i : obs) {
      a.push_back(o_i.id);
      sx.push_back(o_i.x);
      sy.push_back(o_i.y);
    }
	
    //std::cout << a[0] << " " << a[1] << " ..."  << std::endl;
    SetAssociations(p, a, sx, sy);
    
    
    // set weights
    double w = 1.0;
    
    for (auto obs_i : obs) {
      double x = obs_i.x;
      double y = obs_i.y;
      double sigx = std_landmark[0];
      double sigy = std_landmark[1];
      double mux = 0.0;
      double muy = 0.0;
      for (auto lm : predicted) {
       	 if (lm.id == obs_i.id) {
           //std::cout << "lm " << lm.id << std::endl;
           mux = lm.x;
           muy = lm.y;
         }
        //std::cout << "x:" << x << " y:" << y << " mux:" << mux << " muy:" << muy << std::endl;
      }
      
      double prob = exp(-(pow((x-mux),2.0)/(2.0*pow(sigx,2.0)) +  pow((y-muy),2.0) / (2.0*pow(sigy,2.0)) ) );
      prob = prob / (2 * M_PI * sigx * sigy);  
      
      w *= prob;
      
    }
    //std::cout << "w: " << w << std::endl;
    p.weight = w;
    w_norm += w;
  }
  //std::cout << "p.weight: " << p.weight << std::endl;
  //std::cout << "w_norm: " << w_norm << std::endl;
  weights.clear();
  int i = 0;
  for (auto &p : particles) {

    p.weight = p.weight / w_norm;
    if (i==0) {std::cout << p.weight << std::endl; }
    // weights[i] = p.weight;
    weights.push_back(p.weight);
    i++;
  }
  //std::cout << particles[0].weight << std::endl;
  //std::cout << weights[0] << std::endl;


  //std::cout << "sum of weights: " << accumulate(weights.begin(), weights.end(), 0) << std::endl;
}

void ParticleFilter::resample() {
  /**
   * Resample particles with replacement with probability proportional 
   *   to their weight. 
   */
  
  std::default_random_engine gen;
  vector<Particle> p_new;
  
  std::discrete_distribution<int> w_dist(weights.begin(), weights.end());
  
  for (int i = 0; i < num_particles; i++) {
	  p_new.push_back(particles[w_dist(gen)]);
  }
  
  particles = p_new;
  
  
  /* alternative approach: 
  std::default_random_engine gen;

  vector<Particle> p_new;
  
  std::uniform_int_distribution<int> dist_idx(0, num_particles - 1);
  
  int idx = dist_idx(gen);
  double beta = 0.0;
  double max_w = *std::max_element( weights.begin(), weights.end() );
  // std::cout << "max w: " << max_w << std::endl;
  std::uniform_real_distribution<double> dist_u(0.0, 2.0 * max_w);
  
  for (int i = 0; i < num_particles; i++) {
    
    beta = beta + dist_u(gen);
    while(weights[idx] < beta) {
      beta -= weights[idx];
      idx = (idx + 1) % (num_particles - 1);
    }
    
    p_new.push_back(particles[idx]);
  }
  
  particles = p_new;
  */
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}