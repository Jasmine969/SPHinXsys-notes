## Summary

* [SPHinXsys notes of Jasmine Feng](README.md)
* [安装](安装/源码编译安装SPHinXsys.md)
* 案例注解
	* [01 二维溃坝案例注解](案例注解/01_SPHinXsys二维溃坝案例注解.md)
	* [02 二维槽道流案例注解](案例注解/02_SPHinXsys二维槽道流案例注解.md)
	* [03 三维圆管泊肃叶流动案例注解](案例注解/03_SPHinXsys三维圆管泊肃叶流动案例注解.md)
	* [04 二维脉动泊肃叶流动案例注解](案例注解/04_SPHinXsys二维脉动泊肃叶流动案例注解.md)
* 用户指南
    * [快速编译新案例](用户指南/快速编译新案例.md)
    * [常见粒子属性的获取与输出](用户指南/常见粒子属性的获取与输出.md)
    * [几何创建](用户指南/几何创建.md)
    * [指定物体运动或变形](用户指南/指定物体运动或变形.md)
    * [施加外力](用户指南/施加外力.md)
    * [如何在精确的时间步输出粒子信息到VTP文件](用户指南/如何在精确的时间步输出粒子信息到VTP文件.md)
* [算法](算法/算法.md)
    * [边界截断问题的解决](算法/边界截断问题的解决.md)
* 源码剖析

  * [adaptation](源码剖析/adaptation.md)
  * bodies
  	* [base_body](源码剖析/bodies/base_body.md)
  	* [base_body_part](源码剖析/bodies/base_body_part.md)
  * body_relations
  	* [base_body_relation](源码剖析/body_relations/base_body_relation.md)
  * common
  	* [sphinxsys_variable](源码剖析/common/sphinxsys_variable.md)
  * particles
  	* [base_particles](源码剖析/particles/base_particles.md)
  * [particle_dynamics](源码剖析/particle_dynamics.md)
  	* [dynamic algorithm](源码剖析/particle_dynamics/dynamic_algorithms.md)
  	* [fluid_dynamics](源码剖析/particle_dynamics/fluid_dynamics.md)
  	  * [density summation](源码剖析/particle_dynamics/fluid_dynamics/density_summation.md)
  	  * [fluid boundary](源码剖析/particle_dynamics/fluid_dynamics/fluid_boundary.md)
  	  * [fluid integration](源码剖析/particle_dynamics/fluid_dynamics/fluid_integration.md)
  	  * [fluid timestep](源码剖析/particle_dynamics/fluid_dynamics/fluid_timestep.md)
  	  * [transport velocity correction](源码剖析/particle_dynamics/fluid_dynamics/transport_velocity_correction.md)
  	  * [viscous dynamics](源码剖析/particle_dynamics/fluid_dynamics/viscous_dynamics.md)
  	* general_dynamics
  	  * [force_prior](源码剖析/particle_dynamics/general_dynamics/force_prior.md)
  	* relax_dynamics
  	  - [relax_dynamics](源码剖析/particle_dynamics/relax_dynamics/relax_stepping.md)
  * particle_generator
  	* [粒子生成](源码剖析/particle_generator/粒子生成.md)
  * physical_closure
  	- materials
  	  - [elastic_solid](源码剖析/physical_closure/materials/elastic_solid.md)
  * [SPHinXsys 中用到的标签派发技术](源码剖析/SPHinXsys中用到的标签派发技术.md)
  * [从源码到范式：SPHinXsys的RAII所有权设计与指针策略](源码剖析/从源码到范式：SPHinXsys的RAII所有权设计与指针策略.md)
* 其他
	* [写了个SPHinXsys的GDB扩展](其他/写了个SPHinXsys的GDB扩展.md)
	* [geoparticle数据导出到SPHinXsys](其他/geoparticle数据导出到SPHinXsys.md)

