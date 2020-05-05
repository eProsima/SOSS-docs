Getting Started
===============

**Table of Contents**

* :ref:`Installation and deployment`

* :ref:`Example: ROS1-ROS2 communication`

* :ref:`Setting up SOSS`

* :ref:`Running SOSS`

* :ref:`Getting Help`


Installation and deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In this section, we sketch the steps necessary for installing *SOSS* and running a :code:`soss` instance.
Note that the workflow is
dependent on the specific systems that are being communicated, given that each is brought into the *SOSS* world
via a dedicated **System-Handle**.
An example of how these generic steps are implemented in a concrete use-case can be found in the following section.

As a first step, you will need to create a
`colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__
to clone the *SOSS* repository.
This repository contains the *SOSS* core library and the **System-Handles** for some of the protocols that are
integrated into the *SOSS* world.
It consists of many cmake packages which can be configured and built manually, but we recommend to use
a `colcon workspace <https://colcon.readthedocs.io/en/released/user/quick-start.html>`__, which makes the job much
smoother.
To do so, create a :code:`workspace/soss` folder and clone *SOSS* into it:

.. code-block:: bash

    cd
    mkdir -p workspaces/soss
    cd workspaces/soss
    git clone ssh://git@github.com/eProsima/soss_v2 src/soss --recursive -b feature/xtypes-dds

Note: the :code:`--recursive` flag is mandatory to download some required third-parties.

Notice that the *SOSS* repository does not contain all the **System-Handles** of the protocols that are to date
integrated into the *SOSS* world.
For those **System-Handles** that are not built-in, you need to clone their specific repositories into the
:code:`workspace/soss` folder as well.

Here, you can find a table of the repositories where you can find all of the *SOSS*-supported **System-Handles**:

+-----------------------------------------------------------------+-------------------------------------------------------------------------+
| **System-Handle**                                               | Repository                                                              |
+=================================================================+=========================================================================+
| **SOSS-ROS2**, **SOSS-WEBSOCKET**, **SOSS-MOCK**, **SOSS-ECHO** | ssh://git@github.com/eProsima/soss_v2 -b feature/xtypes-dds             |
+-----------------------------------------------------------------+-------------------------------------------------------------------------+
| **SOSS-DDS**                                                    | ssh://git@github.com:eProsima/SOSS-DDS.git -b feature/xtypes-dds        |
+-----------------------------------------------------------------+-------------------------------------------------------------------------+
| **SOSS-ROS1**                                                   | ssh://git@github.com:eProsima/soss-ros1.git -b feature/xtypes-support   |
+-----------------------------------------------------------------+-------------------------------------------------------------------------+
| **SOSS-FIWARE**                                                 | ssh://git@github.com:eProsima/SOSS-FIWARE.git -b feature/xtypes-support |
+-----------------------------------------------------------------+-------------------------------------------------------------------------+

Once all the necessary packages have been cloned, you need to build them. To do so, you can run:

.. code-block:: bash

    colcon build

with the possible addition of flags depending on the specific use-case. Notice also that in most situations you will
have to execute some additional commands before running the :code:`colcon build`, e. g. sourcing an external overlay,
as is the case for *ROS2* and *ROS1* (see e. g. next section).

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

The workspace is now prepared for running a :code:`soss` instance. From the fully overlaid shell, you will have to
execute the :code:`soss` command, followed by the name of the YAML configuration file that describes 
how messages should be passed among the middlewares involved:

.. code-block:: bash
    
    soss <config.yaml>

Once *SOSS* is initiated, the user will be able to communicate the desired protocols.

For more information on how to configure *SOSS* via a YAML file, please refer to :ref:`Yaml Configuration
<yaml configuration>`. For information on how to create your own custom **System-Handle**, see
:ref:`System-Handle Creation <system-handle creation>` instead.

**Note**: the sourcing of the local colcon overlay is required every time the colcon workspace is opened in
a new shell environment.
As an alternative, you can copy the source command with the full path of your local installation to your 
:code:`.bashrc` file. For instance, for *SOSS* you would add:

.. code-block:: bash

    source PATH_TO_WORKSPACE/workspaces/soss/install/setup.bash

Where :code:`PATH_TO_WORKSPACE` is the path to the local :code:`workspaces/soss` directory.
The same applies for the **System-Handle** repositories. 


Example: ROS1-ROS2 communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As a demonstration of *SOSS*' capabilities and usage, we will walk you through how to set up a communication
between *ROS1* and *ROS2*.

Setting up SOSS
---------------

We will assume that you have installed
`ROS1 Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`__ and
`ROS2 Crystal <https://index.ros.org//doc/ros2/Installation/Linux-Install-Debians/#installing-ros2-via-debian-packages>`__
using the ROS PPAs. To run the :code:`soss-ros2-test` integration test, you will also need

.. code-block:: bash

    sudo apt install ros-crystal-test-msgs

Note: the same steps are applicable to *Dashing*.

Create a colcon workspace as explained above

.. code-block:: bash

    cd
    mkdir -p workspaces/soss
    cd workspaces/soss
    git clone ssh://git@github.com/eProsima/soss_v2 src/soss --recursive -b feature/xtypes-dds

and source the *ROS2 Crystal* overlay:

.. code-block:: bash

    source /opt/ros/crystal/setup.bash

Now, you can run:

.. code-block:: bash

    colcon build

If any packages are missing dependencies **causing the compilation to fail**, you can add the flag
:code:`--packages-up-to soss-ros2-test` to make sure that you at least build :code:`soss-ros2-test`:

.. code-block:: bash

    colcon build --packages-up-to soss-ros2-test

Once that's finished building, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash

Notice, with reference to the table above, that you now have both *SOSS* and the **SOSS-ROS2**
**System-Handle** installed.
To get the **SOSS-ROS1** **System-Handle**, you can create a new workspace, and then clone the dedicated
repository into it:

.. code-block:: bash

    cd ..
    mkdir soss-ros1
    cd soss-ros1
    git clone ssh://git@github.com/osrf/soss-ros1 src/soss-ros1 -b feature/xtypes-support

Now source the *ROS Melodic* distribution:

.. code-block:: bash

    source /opt/ros/melodic/setup.bash

You will likely see this message:

.. code-block:: bash

    ROS_DISTRO was set to 'crystal' before. Please make sure that the environment does not mix paths from different
    distributions.

That's okay. The reason is that we have made a previous sourcing of *ROS2* in the same shell, but you will be able
to build :code:`soss-ros1` as long as a *ROS1* distribution was sourced more recently than a
*ROS2* distribution.

Now you can use :code:`colcon build` to build :code:`soss-ros1`:

.. code-block:: bash

    colcon build


And finally, you can source the new colcon overlay:

.. code-block:: bash

    source install/setup.bash


You may see another warning about :code:`ROS_DISTRO`. That's okay.

Running SOSS
------------

After following the above build instructions, **open a new shell** environment and run:

.. code-block:: bash

    source /opt/ros/melodic/setup.bash
    roscore


Then you can return to the shell environment that you were using to build. **If that shell has already been closed**,
then open a new one, return to your :code:`soss-ros1` workspace and source the overlays:

.. code-block:: bash

    cd ~/workspaces/soss-ros1
    source /opt/ros/melodic/setup.bash
    source /opt/ros/crystal/setup.bash
    source ../soss/install/setup.bash
    source install/setup.bash


Now from the fully-overlaid shell, you can run the :code:`soss` instance:

.. code-block:: bash

    soss src/soss-ros1/examples/hello_ros.yaml


In this command, the executable :code:`soss` is given a YAML configuration file to describe how messages
should be passed among whichever middlewares (in this case, *ROS1* and *ROS2*).

In another **new shell environment**, run:

.. code-block:: bash

    source /opt/ros/melodic/setup.bash
    rostopic echo /hello_ros1


In yet another **new shell environment**, run:

.. code-block:: bash

    source /opt/ros/crystal/setup.bash
    ros2 topic echo /hello_ros2 std_msgs/String


Now when you send messages to the topic :code:`/hello_ros1` from *ROS2*, they will appear
in the *ROS1* :code:`rostopic echo` terminal. For example, open a **new shell environment** and run:

.. code-block:: bash

    source /opt/ros/crystal/setup.bash
    ros2 topic pub -r 1 /hello_ros1 std_msgs/String "{data: \"Hello, ros1\"}"


Or you can send messages from *ROS1* to *ROS2*. For example, open a **new shell environment** and run:

.. code-block:: bash

    source /opt/ros/melodic/setup.bash
    rostopic pub -r 1 /hello_ros2 std_msgs/String "Hello, ros2"


Notice that even if this demo requires 6 shell environments to run, *SOSS* itself only occupies
one shell.

Getting Help
^^^^^^^^^^^^

If you need support you can reach us by mail at
`support@eProsima.com <mailto:support@eProsima.com>`__ or by phone at `+34 91 804 34 48 <tel:+34918043448>`__.
