/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal
 *********************************************************************
  \remarks check out http://torjo.com/tobias/

  \file   singleton_factory.h

  \author Mrinal Kalakrishnan, Peter Pastor
  \date   Aug 18, 2011

 *********************************************************************/

#ifndef SINGLETON_FACTORY_H_
#define SINGLETON_FACTORY_H_

#include <string>
#include <tr1/unordered_map>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/utility/singleton.hpp>
#include <usc_utilities/assert.h>

#define BOOST_SINGLETON_TEMPLATE_PLACEMENT_DECLARATION(ClassNameTemplate) \
  friend struct boost::detail::singleton_initialization; \
  static typename ClassNameTemplate::context_type singleton_placement();
#define BOOST_SINGLETON_TEMPLATE_PLACEMENT(Type) \
  template<> \
  Type::context_type Type::singleton_placement() \
  { \
    typedef Type::base_class_type b; \
    return boost::detail::singleton_initialization::call_impl<void, b>(boost::mpl::bool_<false>()); \
  }

namespace usc_utilities
{

// TODO: worry about  lifetime control
// struct singleton_factory_tag;

template<class T>
  class SingletonFactory : public boost::singleton<SingletonFactory<T> >
  {

    /*! Keep constructor private
     */
    friend class boost::singleton<SingletonFactory<T> >;

  public:

    /*!
     * @param topic_name
     * @return
     */
    boost::shared_ptr<T> get(const std::string& topic_name)
    {
      boost::mutex::scoped_lock lock(mutex_);
      typename std::tr1::unordered_map<std::string, boost::shared_ptr<T> >::iterator it;
      it = topic_name_to_instance_map_.find(topic_name);
      if (it == topic_name_to_instance_map_.end())
      {
        topic_name_to_instance_map_.insert(typename std::tr1::unordered_map<std::string, boost::shared_ptr<T> >::value_type(topic_name, boost::shared_ptr<T>(new T())));
        it = topic_name_to_instance_map_.find(topic_name);
        ROS_INFO("Created new singleton on topic >%s<.", topic_name.c_str());
      }
      ROS_ASSERT(it->second);
      return it->second;
    }

    BOOST_SINGLETON_TEMPLATE_PLACEMENT_DECLARATION(SingletonFactory<T>)

  private:

    /*! No construction
     */
    SingletonFactory( boost::restricted) {};
    SingletonFactory(SingletonFactory const&);
    SingletonFactory& operator=(SingletonFactory const&);

    boost::mutex mutex_;
    std::tr1::unordered_map<std::string, boost::shared_ptr<T> > topic_name_to_instance_map_;

  };
}

#endif /* SINGLETON_FACTORY_H_ */
