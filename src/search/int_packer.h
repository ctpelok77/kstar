#ifndef INT_PACKER_H
#define INT_PACKER_H

#include <vector>

/*
  Utility class to pack lots of unsigned integers (called "variables"
  in the code below) with a small domain {0, ..., range - 1}
  tightly into memory. This works like a bitfield except that the
  fields and sizes don't need to be known at compile time.

  For example, if we have 40 binary variables and 20 variables with
  range 4, storing them would theoretically require at least 80 bits,
  and this class would pack them into 12 bytes (three 4-byte "bins").

  Uses a greedy bin-packing strategy to pack the variables, which
  should be close to optimal in most cases. (See code comments for
  details.)
*/

class IntPacker {
public:
    typedef unsigned int Bin;

private:
    struct VariableInfo {
        int range;
        int bin_index;
        int shift;
        Bin read_mask;
        Bin clear_mask;
    };

    std::vector<VariableInfo> var_infos;
    int num_bins;

    void pack_bins(const std::vector<int> &ranges);

public:
    /*
      The constructor takes the range for each variable. The domain
      of variable i is {0, ..., ranges[i] - 1}. Because we are using
      signed ints for the range, a variable can take up at most 31
      bits. Note that some parts of the code depend on the fact
      that a variable cannot use the full 32 bits of a bin.
      (For example, the code might perform bit shifts by 32 bits, whose
      behaviour is not defined by the C++ standard for 32-bit types.)
    */
    explicit IntPacker(const std::vector<int> &ranges);
    ~IntPacker();

    int get(const Bin *buffer, int var) const;
    void set(Bin *buffer, int var, int value) const;

    int get_num_bins() const {return num_bins; }
    std::size_t get_bin_size_in_bytes() const {return sizeof(Bin); }
};

#endif
