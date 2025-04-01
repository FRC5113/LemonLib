import magicbot
import commands2

class CommandMagicRobot(magicbot.MagicRobot):
    """
    Wrapper for the magicbot robot class to allow for command-based
    functionality. This class is used to create a robot that can be
    controlled using commands, while still using the magicbot framework.
    """

    
    def _do_periodics(self):
        super()._do_periodics()
        commands2.CommandScheduler.getInstance().run()
        self.period = max(self.control_loop_wait_time, self.watchdog.getTime())
        print (commands2.CommandScheduler.getInstance()._scheduledCommands)

    def _enabled_periodic(self) -> None:
        """Run components and all periodic methods."""
        watchdog = self.watchdog

        for name, component in self._components:
            if commands2.CommandScheduler.getInstance()._scheduledCommands == {}:
                try:
                    component.execute()
                
                except:
                    self.onException()
            watchdog.addEpoch(name)

        self._do_periodics()

        for reset_dict, component in self._reset_components:
            component.__dict__.update(reset_dict)