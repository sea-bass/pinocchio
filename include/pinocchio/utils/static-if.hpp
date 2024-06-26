//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_utils_static_if_hpp__
#define __pinocchio_utils_static_if_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{
  namespace internal
  {

    enum ComparisonOperators
    {
      LT,
      LE,
      EQ,
      GE,
      GT
    };

    template<typename LhsType, typename RhsType, typename ThenType, typename ElseType>
    struct if_then_else_impl;

    template<typename LhsType, typename RhsType>
    struct comparison_eq_impl;

    /// \brief Template specialization for  similar return types;
    template<typename LhsType, typename RhsType, typename return_type>
    struct traits<if_then_else_impl<LhsType, RhsType, return_type, return_type>>
    {
      typedef return_type ReturnType;
    };

    template<typename condition_type, typename ThenType, typename ElseType>
    struct if_then_else_impl<condition_type, condition_type, ThenType, ElseType>
    {
      typedef typename internal::traits<if_then_else_impl>::ReturnType ReturnType;

      static inline ReturnType run(
        const ComparisonOperators op,
        const condition_type & lhs_value,
        const condition_type & rhs_value,
        const ThenType & then_value,
        const ElseType & else_value)
      {
        switch (op)
        {
        case EQ:
          if (lhs_value == rhs_value)
            return then_value;
          else
            return else_value;
          break;
        case LT:
          if (lhs_value < rhs_value)
            return then_value;
          else
            return else_value;
          break;
        case LE:
          if (lhs_value <= rhs_value)
            return then_value;
          else
            return else_value;
          break;
        case GE:
          if (lhs_value >= rhs_value)
            return then_value;
          else
            return else_value;
          break;
        case GT:
          if (lhs_value > rhs_value)
            return then_value;
          else
            return else_value;
          break;
        }
        PINOCCHIO_THROW_PRETTY(
          std::logic_error, "ComparisonOperators " << static_cast<int>(op) << " is not managed");
      }
    };

    template<typename LhsType, typename RhsType, typename ThenType, typename ElseType>
    inline typename if_then_else_impl<LhsType, RhsType, ThenType, ElseType>::ReturnType
    if_then_else(
      const ComparisonOperators op,
      const LhsType & lhs_value,
      const RhsType & rhs_value,
      const ThenType & then_value,
      const ElseType & else_value)
    {
      typedef if_then_else_impl<LhsType, RhsType, ThenType, ElseType> algo;
      return algo::run(op, lhs_value, rhs_value, then_value, else_value);
    }

    // Generic
    template<typename LhsType, typename RhsType>
    struct comparison_eq_impl
    {
      static inline bool run(const LhsType & lhs_value, const RhsType & rhs_value)
      {
        return lhs_value == rhs_value;
      }
    };

    template<typename condition_type>
    struct comparison_eq_impl<condition_type, condition_type>
    {
      static inline bool run(const condition_type & lhs_value, const condition_type & rhs_value)
      {
        return lhs_value == rhs_value;
      }
    };

    template<typename LhsType, typename RhsType>
    inline bool comparison_eq(const LhsType & lhs_value, const RhsType & rhs_value)
    {
      typedef comparison_eq_impl<LhsType, RhsType> algo;
      return algo::run(lhs_value, rhs_value);
    }
  } // namespace internal
} // namespace pinocchio

#endif
