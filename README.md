
# Sly Physics

A ECS Bevy physics plugin.

Most likely you should go use rapier, this is a moon shot.

This is where I am teaching myself what goes into a physiscs engine, the goal is have a working physiscs engine in one crate, thats good only for games, and easy to use.  Hope to focusing on performance.

# Why

I have playing with bevy for a while now and the only real option you have for physics is [Rapier](https://github.com/dimforge/rapier), either though [bevy_rapier](https://github.com/dimforge/bevy_rapier) or [heron](https://github.com/jcornaz/heron) or Physx. Physx is a no for wasm support and removing unsafe things.  While rapier is truly amazing, I wanted a bevy native experence using the ecs that I can understand and hack on in one repo.

# Status



 - [x] Colliders
   - [ ] Sphere
   - [ ] Cu
 - [x] Phases
   - [x] Broad - prune_and_sweep, working on bvh
   - [x] Narrow
     - [x] Continouns
       - [x] Sphere vs Sphere
       - [ ] Box, using gj
       - [x] GJK
      - [ ] 
## Credit

- [Game Physics in One Weekend](https://gamephysicsweekend.github.io/)
  - This Engine is based on the 3 book series, you can  can get all 3 for $9 on Amazon Kindle.
- [Bitshifter](https://github.com/bitshifter)
  - The 3 books have already been ported to bevy by [Bitshifter](https://github.com/bitshifter) in a repo [here](https://github.com/bitshifter/bevy-physics-weekend)
  - Bitshifter is the developer of the [glam](https://github.com/bitshifter/glam-rs) crate, finding that repo and knowing I can use it as ref gave me the confidence to even start this.
- [bevy_rapier](https://github.com/dimforge/bevy_rapier)
- [BVH tutorial series](https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/) by Jacco Bikker.   Go check it out if bvh's interest you. 