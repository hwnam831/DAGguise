/*
 * Copyright (c) 2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Hansson
 */

/**
 * @file
 * DRAMSim2
 */
#ifndef __MEM_DRAMSIM2_HH__
#define __MEM_DRAMSIM2_HH__

#include <fstream>
#include <queue>
#include <random>
#include <unordered_map>

#include "debug/DefensiveML.hh"
#include "mem/abstract_mem.hh"
#include "mem/dramsim2_wrapper.hh"
#include "mem/qport.hh"
#include "params/DRAMSim2.hh"

class AMLShaper{
    float weight0[16][16];
    float bias0[16];

    float weight1[16][16];
    float bias1[16];

    float avg_scores[16];

    float amp=8.0;


    std::default_random_engine generator;
    public:
        AMLShaper(std::string filename, float amplitude){
            amp = amplitude;
            srand(time(NULL));
            generator.seed(rand()%100);
            if (filename != ""){

              std::ifstream paramfile(filename);
              std::string line;
              std::getline(paramfile, line);
              //std::cout << line << std::endl;
              for (int o=0; o<16; o++){
                  for (int i=0; i<16; i++){
                      paramfile >> weight0[o][i];
                      //std::cout << weight0[o][i] << " ";
                  }
                  //std::cout << std::endl;
              }
              //std::cout << std::endl;
              std::getline(paramfile, line);
              std::getline(paramfile, line);
              //std::cout << line << std::endl;
              for (int i=0; i<16; i++){
                  paramfile >> bias0[i];
                  //std::cout << bias0[i] << " ";
              }
              //std::cout << std::endl;
              std::getline(paramfile, line);
              std::getline(paramfile, line);
              //std::cout << line << std::endl;
              for (int o=0; o<16; o++){
                  for (int i=0; i<16; i++){
                      paramfile >> weight1[o][i];
                      //std::cout << weight1[o][i] << " ";
                  }
                  //std::cout << std::endl;
              }
              paramfile.close();
            }



        }

        int argmax(float x[16]){
            int max = x[0];
            int idx = 0;
            for (int i=1; i<16; i++){
                if (x[i] > max){
                    max = x[i];
                    idx = i;
                }
            }
            return idx;
        }

        void fflayer(float x[16], float w[16][16],
                    float b[16], float* out_buffer){

            for (int o=0; o<16; o++){
                out_buffer[o] = b[o];
                for (int i=0; i<16; i++){
                    out_buffer[o] += x[i]*w[o][i];
                }
            }
        }
        void forward(std::deque<int64_t> inq,
                    int64_t* target_buffer){
            float input[16];
            for (int i=0; i<16; i++){
                input[i] = (inq.at(i) - 40000.0)/6000;
            }

            float x[16];

            fflayer(input, weight0, bias0, x);

            for (int i=0; i<16; i++){
                x[i] = x[i] > 0 ? x[i] : 0;
            }

            float x2[16];

            fflayer(x, weight1, bias1, x2);

            float level = (amp * argmax(x2) / 16.0);

            std::normal_distribution<float> distribution(level,level/2);
            for (int i=0; i<8; i++){
                float p = distribution(generator);
                int target = p*6000 + 40000;
                target_buffer[i] = target > 0 ? target : 0;
            }


        }

};
class DRAMSim2 : public AbstractMemory
{
  private:

    /**
     * The memory port has to deal with its own flow control to avoid
     * having unbounded storage that is implicitly created in the port
     * itself.
     */
    class MemoryPort : public SlavePort
    {

      private:

        DRAMSim2& memory;

      public:

        MemoryPort(const std::string& _name, DRAMSim2& _memory);

      protected:

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        bool recvTimingReq(PacketPtr pkt);

        void recvRespRetry();

        AddrRangeList getAddrRanges() const;

    };

    struct AMLStats : public Stats::Group {
        AMLStats(Stats::Group *parent);

        /** Count the number of dropped requests. */
        Stats::Scalar totalPerturb;

        /** Total num of ticks read reqs took to complete  */
        Stats::Scalar totalReadLatency;


        /** Count the number reads. */
        Stats::Scalar totalReads;

        /** Avg num of ticks each read req took to complete  */
        Stats::Formula avgReadLatency;

        Stats::Formula avgPerturb;

    } stats;

    MemoryPort port;

    /**
     * The actual DRAMSim2 wrapper
     */
    DRAMSim2Wrapper wrapper;

    /**
     * Is the connected port waiting for a retry from us
     */
    bool retryReq;

    /**
     * Are we waiting for a retry for sending a response.
     */
    bool retryResp;

    /**
     * Keep track of when the wrapper is started.
     */
    Tick startTick;

    /**
     * AML defense policy
     */
    enum DefensePolicy{
      NO_DEFNSE,
      AML_DEFENSE,
      PAD_DEFENSE
    };
    DefensePolicy policy;
    /**
     * Keep track of what packets are outstanding per
     * address, and do so separately for reads and writes. This is
     * done so that we can return the right packet on completion from
     * DRAMSim.
     */
    std::unordered_map<Addr, std::queue<PacketPtr> > outstandingReads;
    std::unordered_map<Addr, std::queue<PacketPtr> > outstandingWrites;
    std::unordered_map<Addr, std::queue<Tick> > readEntryTimes;
    /**
     * Count the number of outstanding transactions so that we can
     * block any further requests until there is space in DRAMSim2 and
     * the sending queue we need to buffer the response packets.
     */
    unsigned int nbrOutstandingReads;
    unsigned int nbrOutstandingWrites;

    /**
     * Queue to hold response packets until we can send them
     * back. This is needed as DRAMSim2 unconditionally passes
     * responses back without any flow control.
     */
    std::deque<PacketPtr> responseQueue;
    std::deque<int64_t> readLatencies;
    int64_t targetLatencies[8];
    int targetPosition;

    AMLShaper shaper;

    unsigned int nbrOutstanding() const;

    /**
     * When a packet is ready, use the "access()" method in
     * AbstractMemory to actually create the response packet, and send
     * it back to the outside world requestor.
     *
     * @param pkt The packet from the outside world
     */
    void accessAndRespond(PacketPtr pkt, Tick perturb=0);

    void sendResponse();

    /**
     * Event to schedule sending of responses
     */
    EventFunctionWrapper sendResponseEvent;

    /**
     * Progress the controller one clock cycle.
     */
    void tick();

    /**
     * Event to schedule clock ticks
     */
    EventFunctionWrapper tickEvent;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

  public:

    typedef DRAMSim2Params Params;
    DRAMSim2(const Params *p);

    /**
     * Read completion callback.
     *
     * @param id Channel id of the responder
     * @param addr Address of the request
     * @param cycle Internal cycle count of DRAMSim2
     */
    void readComplete(unsigned id, uint64_t addr, uint64_t cycle);

    /**
     * Write completion callback.
     *
     * @param id Channel id of the responder
     * @param addr Address of the request
     * @param cycle Internal cycle count of DRAMSim2
     */
    void writeComplete(unsigned id, uint64_t addr, uint64_t cycle);

    DrainState drain() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void init() override;
    void startup() override;

    void startDefence(uint64_t cpuid, uint64_t iDefenceDomain, uint64_t dDefenceDomain);
    void updateDefence(uint64_t oldDomain, uint64_t newDomain, bool isdata);
    void endDefence();

  protected:

    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();

};

#endif // __MEM_DRAMSIM2_HH__
