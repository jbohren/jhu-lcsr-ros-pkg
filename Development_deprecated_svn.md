## Introduction ##
This page includes development guidelines and useful practices, including [a review of branching with svn](http://code.google.com/p/jhu-lcsr-ros-pkg/wiki/Development#Branching_HOWTO).

## Table Of Contents ##


## Repository Layout ##

The repository contains both loose ROS packages and ROS packages that are children of a given stack. Loose ROS packages are stored in [/svn/pkgs](http://code.google.com/p/jhu-lcsr-ros-pkg/source/browse/#svn%2Fpkgs) and ROS stacks are stored in [/svn/stacks](http://code.google.com/p/jhu-lcsr-ros-pkg/source/browse/#svn%2Fstacks). Note that if code meant to be supported, it should be in a _stack_ and versioned accordingly.

Note that the entire collection of loose packages shares common tags, branches, and trunk directories, whereas each stack has its own set.

## Checking Out for Development ##

Checking out the repository is explained [here](http://code.google.com/p/jhu-lcsr-ros-pkg/source/checkout). Commit access is given through secure http (https), and Google Code will generate a random password for you to use with your Google Code account. If you have a secure-password caching system like the [Gnome Keyring](http://live.gnome.org/GnomeKeyring), and have not disabled password caching, svn should cache this password.

## Loose Package Development ##

ROS packages that are intended to be shared, but not necessarily kept to a strict versioning cycle, and do not immediately belong to any stack can be stored as loose packages. These packages are labeled as _UNSTABLE_ on their development pages. Development practices for these packages is basically up to the whim of the package's owner.

After a period of time, loose packages might become broken, and if the maintainer has abandoned them, they may be moved to a "deprecated" or "stale" directory in the repository.

## Stack Development ##
ROS packages that are intended to be shared, but also provide some level of stability (at least with respect to versioning), should be combined into _stacks_.

### Stack Guidelines ###
Stack development practices aim to maintain stability for the users of the stacks. There are a few rules / guidelines to operate by (_note that these rules are still in development_)

  * A stack's trunk should _always_ build. In other words, _trunk_ should be considered _UNSTABLE_ but not _broken_.
  * If you are going to break the build, do it in a branch, and only merge back into trunk when the branch builds.
  * If you are going to change the API, do it in a branch, and merge back into trunk when the changes are completed and documented.

### Stack Versioning ###
Versioning is a tricky thing, here we will try to follow the suggested stack version policy as given on ros.org, found here: http://www.ros.org/wiki/StackVersionPolicy. Note that this is a rigorous process, but it has proved successful in the field. This policy is summarized here for convenience. Note that we will not be as rigorous with user testing since we do not have the resources to do so.

Every stack version is numbered with three numbers: major.minor.patch. As the stack matures, it will go through three phases:
  * pre-1.0.0: _unstable/experimental_
  * 1.0.0: _initial fully-supported release_
  * post-1.0.0: _stable odd/even process_

When stacks versions are tagged, they are tagged like "stack\_name-x.y.z".

#### Pre-1.0.0 ####
  * 0.1.x: completely experimental, unreviewed
  * 0.2.x: some review has occurred, and we are migrating this stack to more stable APIs
  * 0.3.x: ready to start being used in research, though definitely not stable
  * 0.4.x-0.8.x: on stable development cycle towards 1.0 release (aka tick-tock)
  * 0.9.x: 1.0.0 release candidate

#### 1.0.0 ####
A 1.0.0 stack usually takes a long development cycle, since it includes all the stages described in the previous section. Stacks have historically taken on the order of a year to go from 0.1.0 to 1.0.0. Pushing to 1.0.0 implies that the stack's maintainers:
  * are ready to commit to supporting the APIs in the stack
  * have thoroughly tested the code and can be confident it is free of bugs
  * have documented the features and API of the stack
  * have made sure the stack only depends on other stable software

#### Post-1.0.0 ####
After a stack has reached 1.0.0 status, we can follow an "odd/even" development cycle, where "odd" and "even" refer to the minor version.

During an _odd cycle_, new features are developed, tested, and refined, and during an _even cycle_, the pre-1.0.0 development process is followed.

If a dramatic change in the API is made, or enough even/odd cycles have happened, the major version is incremented.

## Branching HOWTO ##
Branching in svn is not scary (any more). Branches are just copies of code in one part of the repository to some other part of the repository.

### Creating a Branch ###
To make a development branch of a stack's trunk on the server:
```
$> svn copy https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/trunk\
https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/branches/some_stack-an_id\
-m "Creating branch some_stack-an_id"
```
Depending on the purpose of the branch, "an\_id" can be something like "feature\_name" or even just your name if you want a personal playground.

### Keeping your branch up to date ###
Note that the longer you go without merging your branch back into trunk, the higher the chance is that trunk will drift too far from what it was when you branched from it. You can prevent this by frequently merging changes to trunk into your branch.

Check out your branch (if necessary) and navigate into it:
```
$> svn checkout https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/branches/some_stack-an_id
$> cd some_stack-an_id
$> svn up
```

Merge changes made in trunk (since you branched) into your branch. Note this will only change the working copy:
```
$> svn merge https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/trunk
```

Resolve conflicts appropriately, and commit the changes merged from trunk:
```
$> svn commit -m "Merging changes from trunk into some_stack-an_id branch"
```

### Merging your branch back into trunk ###
Once you are satisfied with the changes you've made to your branch, you can merge your changes back into trunk, and made sure your branch [has the latest changes from trunk](http://code.google.com/p/jhu-lcsr-ros-pkg/wiki/Development#Keeping_your_branch_up_to_date) (see the previous section). Once you've committed the last changes to your branch, this is done like so:

Check out the trunk (if necessary) and navigate into it:
```
$> svn checkout https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/trunk some_stack-trunk
$> cd some_stack-trunk
$> svn up
```

Apply the changes from your branch to the local checkout of trunk:
```
$> svn merge --reintegrate https://jhu-lcsr-ros-pkg.googlecode.com/svn/stacks/some_stack/branches/some_stack-an_id 
```

After you have resolved any conflicts, commit the changes to trunk:
```
$> svn commit -m "Merging some_stack-an_id back into trunk"
```