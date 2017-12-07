//
// Copyright (c) 2015-2017 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_rnea_hxx__
#define __se3_rnea_hxx__

/// @cond DEV

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace se3
{
  struct RneaForwardStep : public fusion::JointVisitor<RneaForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
			    se3::Data &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &,
			    const Eigen::VectorXd &
			    > ArgsType;

    JOINT_VISITOR_INIT(RneaForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      data.a_gf[i] = jdata.S()*jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }

  };

  struct RneaBackwardStep : public fusion::JointVisitor<RneaBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    JOINT_VISITOR_INIT(RneaBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.tau)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
    }
  };

  inline const Eigen::VectorXd&
  rnea(const Model & model, Data& data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;

    for( Model::JointIndex i=1;i<(Model::JointIndex)model.njoints;++i )
    {
      RneaForwardStep::run(model.joints[i],data.joints[i],
                           RneaForwardStep::ArgsType(model,data,q,v,a));
    }
    
    for( Model::JointIndex i=(Model::JointIndex)model.njoints-1;i>0;--i )
    {
      RneaBackwardStep::run(model.joints[i],data.joints[i],
                            RneaBackwardStep::ArgsType(model,data));
    }

    return data.tau;
  }
  
  inline const Eigen::VectorXd &
  rnea(const Model & model, Data & data,
       const Eigen::VectorXd & q,
       const Eigen::VectorXd & v,
       const Eigen::VectorXd & a,
       const container::aligned_vector<Force> & fext)
  {
    assert(fext.size() == model.joints.size());
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero();
    data.a_gf[0] = -model.gravity;
    
    for( Model::JointIndex i=1;i<(Model::JointIndex)model.njoints;++i )
    {
      RneaForwardStep::run(model.joints[i],data.joints[i],
                           RneaForwardStep::ArgsType(model,data,q,v,a));
      data.f[i] -= fext[i];
    }
    
    for( Model::JointIndex i=(Model::JointIndex)model.njoints-1;i>0;--i )
    {
      RneaBackwardStep::run(model.joints[i],data.joints[i],
                            RneaBackwardStep::ArgsType(model,data));
    }
    
    return data.tau;
  }
  
  struct NLEForwardStep : public fusion::JointVisitor<NLEForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(NLEForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q,v);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[(size_t) parent]);
      
      data.a_gf[i]  = jdata.c() + (data.v[i] ^ jdata.v());
      data.a_gf[i] += data.liMi[i].actInv(data.a_gf[(size_t) parent]);
      
      data.f[i] = model.inertias[i]*data.a_gf[i] + model.inertias[i].vxiv(data.v[i]); // -f_ext
    }
    
  };
  
  struct NLEBackwardStep : public fusion::JointVisitor<NLEBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  >  ArgsType;
    
    JOINT_VISITOR_INIT(NLEBackwardStep);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent  = model.parents[i];
      
      jmodel.jointVelocitySelector(data.nle)  = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[(size_t) parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  inline const Eigen::VectorXd &
  nonLinearEffects(const Model & model, Data & data,
                   const Eigen::VectorXd & q,
                   const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.v[0].setZero ();
    data.a_gf[0] = -model.gravity;
    
    for( size_t i=1;i<(size_t) model.njoints;++i )
    {
      NLEForwardStep::run(model.joints[i],data.joints[i],
                          NLEForwardStep::ArgsType(model,data,q,v));
    }
    
    for( size_t i=(size_t) (model.njoints-1);i>0;--i )
    {
      NLEBackwardStep::run(model.joints[i],data.joints[i],
                           NLEBackwardStep::ArgsType(model,data));
    }
    
    return data.nle;
  }
  
  struct computeGeneralizedGravityForwardStep : public fusion::JointVisitor<computeGeneralizedGravityForwardStep>
  {
    typedef boost::fusion::vector< const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &
    > ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityForwardStep);
    
    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.calc(jdata.derived(),q);
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      
      data.a_gf[i] = data.liMi[i].actInv(data.a_gf[(size_t) parent]);
      data.f[i] = model.inertias[i]*data.a_gf[i];
    }
    
  };
  
  struct computeGeneralizedGravityBackwardStep : public fusion::JointVisitor<computeGeneralizedGravityBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &
    >  ArgsType;
    
    JOINT_VISITOR_INIT(computeGeneralizedGravityBackwardStep);
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      
      jmodel.jointVelocitySelector(data.g) = jdata.S().transpose()*data.f[i];
      if(parent>0) data.f[(size_t) parent] += data.liMi[i].act(data.f[i]);
    }
  };
  
  inline const Eigen::VectorXd &
  computeGeneralizedGravity(const Model & model, Data & data,
                            const Eigen::VectorXd & q)
  {
    assert(model.check(data) && "data is not consistent with model.");
    
    data.a_gf[0] = -model.gravity;
    
    for(size_t i=1;i<(size_t) model.njoints;++i)
    {
      computeGeneralizedGravityForwardStep::run(model.joints[i],data.joints[i],
                                     computeGeneralizedGravityForwardStep::ArgsType(model,data,q));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      computeGeneralizedGravityBackwardStep::run(model.joints[i],data.joints[i],
                                      computeGeneralizedGravityBackwardStep::ArgsType(model,data));
    }
    
    return data.g;
  }
  
  struct CoriolisMatrixForwardStep : public fusion::JointVisitor<CoriolisMatrixForwardStep>
  {
    typedef boost::fusion::vector<const se3::Model &,
    se3::Data &,
    const Eigen::VectorXd &,
    const Eigen::VectorXd &
    > ArgsType;

    JOINT_VISITOR_INIT(CoriolisMatrixForwardStep);

    template<typename JointModel>
    static void algo(const se3::JointModelBase<JointModel> & jmodel,
                     se3::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const se3::Model & model,
                     se3::Data & data,
                     const Eigen::VectorXd & q,
                     const Eigen::VectorXd & v)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Inertia::Matrix6 & tmp = data.Itmp;
      
      jmodel.calc(jdata.derived(),q,v);

      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent>0) data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else data.oMi[i] = data.liMi[i];
      
      data.v[i] = jdata.v();
      if(parent>0) data.v[i] += data.liMi[i].actInv(data.v[parent]);
      
      // computes S expressed at the world frame
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S()); // collection of S expressed at the world frame
      
      // computes vxS expressed at the world frame
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      tmp.leftCols(jmodel.nv()) = data.v[i].cross(jdata.S());
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      motionSet::se3Action(data.oMi[i],tmp.leftCols(jmodel.nv()),dJcols);

      // express quantities in the world frame
      data.oYo[i] = data.oMi[i].act(model.inertias[i]);
      
      // computes vxI
      const Motion ov(data.oMi[i].act(data.v[i])); // v_i expressed in the world frame
      Inertia::vxi(ov,data.oYo[i],data.vxI[i]);
    }

  };

  struct CoriolisMatrixBackwardStep : public fusion::JointVisitor<CoriolisMatrixBackwardStep>
  {
    typedef boost::fusion::vector<const Model &,
    Data &
    > ArgsType;

    JOINT_VISITOR_INIT(CoriolisMatrixBackwardStep);

    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      const Model::JointIndex & i = jmodel.id();
      const Model::JointIndex & parent = model.parents[i];
      Inertia::Matrix6 & tmp = data.Itmp;
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Data::Matrix6x>::Type ColsBlock;
      ColsBlock dJcols = jmodel.jointCols(data.dJ);
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      rhsInertiaMult(data.oYo[i],dJcols,jmodel.jointCols(data.dFdv));
      jmodel.jointCols(data.dFdv) += data.vxI[i] * Jcols;
      
      data.C.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i])
      = Jcols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);

      lhsInertiaMult(data.oYo[i],Jcols.transpose(),tmp.topRows(jmodel.nv()));
      for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) = tmp.topRows(jmodel.nv()) * data.dJ.col(j);

      tmp.topRows(jmodel.nv()) = Jcols.transpose() * data.vxI[i];
      for(int j = data.parents_fromRow[(Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(Model::Index)j])
        data.C.middleRows(jmodel.idx_v(),jmodel.nv()).col(j) += tmp.topRows(jmodel.nv()) * data.J.col(j);

      if(parent>0)
      {
        data.oYo[parent] += data.oYo[i];
        data.vxI[parent] += data.vxI[i];
      }
    }
    
    template<typename Min, typename Mout>
    static void rhsInertiaMultVector(const Inertia & Y,
                                     const Eigen::MatrixBase<Min> & m,
                                     const Eigen::MatrixBase<Mout> & f)
    {
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Min,6);
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Mout,6);
      Mout & f_ = const_cast<Mout &>(f.derived());

      f_.template segment<3>(Inertia::LINEAR) = -Y.mass() * Y.lever().cross(m.template segment<3>(Motion::ANGULAR));

      f_.template segment<3>(Inertia::ANGULAR) = Y.inertia() * m.template segment<3>(Motion::ANGULAR);
      f_.template segment<3>(Inertia::ANGULAR) += Y.lever().cross(f_.template segment<3>(Inertia::LINEAR));
      f_.template segment<3>(Inertia::ANGULAR) += Y.mass() * Y.lever().cross(m.template segment<3>(Motion::LINEAR));

      f_.template segment<3>(Inertia::LINEAR) += Y.mass() * m.template segment<3>(Motion::LINEAR);
    }
    
    template<typename Min, typename Mout>
    static void rhsInertiaMult(const Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      assert(J.cols() == F.cols());
      Mout & F_ = const_cast<Mout &>(F.derived());
      
      for(int i = 0; i < J.cols(); ++i)
      {
        rhsInertiaMultVector(Y,J.col(i),F_.col(i));
      }
      
    }
    
    template<typename Min, typename Mout>
    static void lhsInertiaMult(const Inertia & Y,
                               const Eigen::MatrixBase<Min> & J,
                               const Eigen::MatrixBase<Mout> & F)
    {
      Mout & F_ = const_cast<Mout &>(F.derived());
      rhsInertiaMult(Y,J.transpose(),F_.transpose());
    }
  };
  
  inline const Eigen::MatrixXd &
  computeCoriolisMatrix(const Model & model, Data & data,
                        const Eigen::VectorXd & q,
                        const Eigen::VectorXd & v)
  {
    assert(model.check(data) && "data is not consistent with model.");
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    
    for(size_t i=1;i<(size_t) model.njoints;++i)
    {
      CoriolisMatrixForwardStep::run(model.joints[i],data.joints[i],
                                     CoriolisMatrixForwardStep::ArgsType(model,data,q,v));
    }
    
    for(size_t i=(size_t) (model.njoints-1);i>0;--i)
    {
      CoriolisMatrixBackwardStep::run(model.joints[i],data.joints[i],
                                      CoriolisMatrixBackwardStep::ArgsType(model,data));
    }
    
    return data.C;
  }
  
} // namespace se3

/// @endcond

#endif // ifndef __se3_rnea_hxx__
