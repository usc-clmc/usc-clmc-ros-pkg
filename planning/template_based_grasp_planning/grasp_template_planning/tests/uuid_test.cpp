#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/thread.hpp>
#include <vector>
#include <omp.h>

int main() {
	 int num_per_thread = 1000;
	  static boost::mutex mutex_;

	std::vector<boost::uuids::uuid> glob_vec;
	glob_vec.resize(omp_get_max_threads() * num_per_thread, boost::uuids::uuid());
	std::cout << omp_get_max_threads() << ", " << num_per_thread << ", " << "vec_size = " << glob_vec.size() << std::endl;

#pragma omp parallel shared(glob_vec, mutex_)
	{
		for(int i=0; i<num_per_thread; i++)
		{
//			boost::mutex::scoped_lock lock(mutex_);
			boost::uuids::uuid uuid = boost::uuids::random_generator()();

			  std::cout << "THREAD" << omp_get_thread_num() << ": " << uuid <<": " << std::endl;
			int index = omp_get_thread_num() * num_per_thread + i;
			std::cout << omp_get_thread_num() << ", " << num_per_thread << "index = " << index << std::endl;
			std::cout << "vec_size = " << glob_vec.size() << std::endl;
			  glob_vec[index] = uuid;
		}
	}

  for(int i = 0; i < glob_vec.size(); i++)
	  for(int j = i+1; j < glob_vec.size(); j++)
	  {
		  if(glob_vec[i] == glob_vec[j])
		  {
			  std::cout << "FAILURE" << std::endl;
		  }
	  }
  std::cout << "DONE " << omp_get_num_threads() << std::endl;
}
