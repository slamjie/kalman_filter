#include <ros/ros.h>
#include <.h>

struct Particle
{
	double x;
	double y;
	double theta;
	double weight;	
};

class pf
{
public:
	pf()is_initialized(false)
	{

	}
	~pf();

	void init(double x, double y, double theta, double std[])
	{
		num_particles = 100;
		particles.resize(num_particles);

		std::default_random_engine gen;
		std::normal_distribution<double> dist_x(x, std[0]);
	    std::normal_distribution<double> dist_y(y, std[1]);
    	std::normal_distribution<double> dist_theta(theta, std[2]);

    	for(size_t i = 0; i<num_particles;++i)
    	{
    		particles.at(i).x = dist_x(gen);
    		particles.at(i).y = dist_y(gen);
    		particles.at(i).theta = dist_theta(gen);
    		particles.at(i).weight = 1.0;

    	}

    	is_initialized = true;
	}

	void prediction(double delta_t, double std_pos[], double v, double w)
	{
		std::default_random_engine gen;
		std::normal_distribution<double> noisy_x(x, std_pos[0]);
	    std::normal_distribution<double> noisy_y(y, std_pos[1]);
    	std::normal_distribution<double> noisy_theta(theta, std_pos[2]);

    	for (int i = 0; i < num_particles; ++i)
    	{
    		double x0 = particles.at(i).x;
    		double y0 = particles.at(i).y;
    		double theta0 = particles.at(i).theta;

    		double x_pre = 0.0;
    		double y_pre = 0.0;
    		double theta_pre = 0.0;

    		if (fabs(w) < 0.000001)
    		{
    			x_pre = x0 + v * delta_t * cos(theta0);
    			y_pre = y0 + v * delta_t * sin(theta0);
    			theta_pre = theta0;
    		}else{
    			x_pre = x0 + v/w * (sin(theta0 + w * delta_t) - sin(theta0));
    			y_pre = y0 + v/w * (cos(theta0) - cos(theta0 + w * delta_t));
    			theta_pre = theta0 + w * delta_t;
    		}

    		// theta (0~2PI)
    		while(theta_pre > 2 * M_PI)
    			theta_pre -= 2 * M_PI;
    		while(theta_pre < 0.0)
    			theta_pre += 2 * M_PI;

    		particles.at(i).x = x_pre + noisy_x(gen);
    		particles.at(i).y = y_pre + noisy_y(gen);
    		particles.at(i).theta = theta_pre + noisy_theta(gen);
    	}
	}

	void dataAssociation()
	{
		
	}

	void UpdateWeight()
	{

		
	}

private:
	int num_particles;
	std::vector<Particle> particles;
	bool is_initialized;
};