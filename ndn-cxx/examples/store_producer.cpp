#define _CRT_SECURE_NO_WARNINGS
#include<iostream>
#include "face.hpp"
#include "security/key-chain.hpp"
#include <fstream>
#include "util/scheduler.hpp"
#include "time.h"//获取时间的头文件
#include "sstream"
#include "unistd.h"

using namespace std;

static size_t Offset;
static size_t End;
static std::string flowname;
const size_t MAX_SEG_SIZE=4096;
namespace ndn {

 namespace examples {

class Producer : noncopyable
{
public:
  void
  run()
  {
    m_face.setInterestFilter("/example/testApp",
                             bind(&Producer::onInterest, this, _1, _2),
                             RegisterPrefixSuccessCallback(),
                             bind(&Producer::onRegisterFailed, this, _1, _2));
    m_face.processEvents();

  }

private:
  void
  onInterest(const InterestFilter& filter, const Interest& interest)
  {
    //test for producer
    Offset = interest.getFC();
    End = interest.getEC();
   // size_t segNum = End - Offset;
    flowname = interest.getName().toUri(); //test for flowname

    //cout test for New Field 
    std::cout << "<< I: " << interest << std::endl;
    std::cout << "<< New Field: " << interest.getFC() << "(FC) " << interest.getEC() << "(EC)" << std::endl;
   while(Offset < End +1)
   {

                static const std::string content="hello kitty";
                shared_ptr<Data> data = make_shared<Data>(Name(flowname).appendSegment(Offset));
                //data->setName(dataName);
                data->setFreshnessPeriod(time::seconds(10));
                data->setContent(reinterpret_cast<const uint8_t*>(content.c_str()), content.size());

                // Sign Data packet with default identity
                m_keyChain.sign(*data);
                // m_keyChain.sign(data, <identityName>);
                // m_keyChain.sign(data, <certificate>);

                // Return Data packet to the requester
                std::cout << ">> D: " << *data << std::endl;
                m_face.put(*data);
                Offset++;
            }
  }

  void
  onRegisterFailed(const Name& prefix, const std::string& reason)
  {
    std::cerr << "ERROR: Failed to register prefix \""
              << prefix << "\" in local hub's daemon (" << reason << ")"
              << std::endl;
    m_face.shutdown();
  }

private:
  //rewrite by lv;
  Face m_face;
  KeyChain m_keyChain;
};

} // namespace examples
} // namespace ndn

int
main(int argc, char** argv)
{
  ndn::examples::Producer producer;
  try {
    producer.run();
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
  }
  return 0;}
