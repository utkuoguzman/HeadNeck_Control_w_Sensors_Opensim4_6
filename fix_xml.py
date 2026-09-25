with open('test_scripts/fig3_ramp_stretches.py', 'r') as f:
    content = f.read()

import re
old_xml = '''        spindle_ctrl = f"""
        <SpindleController name="spindle_controller">
            <socket_actuators>muscle</socket_actuators>
            <ControlFunctions>
                <Constant>
                    <value>0.1</value>
                </Constant>
            </ControlFunctions>
            <SpindleFunctionsStatic>
                <Constant>
                    <value>{gamma['stat']}</value>
                </Constant>
            </SpindleFunctionsStatic>
            <SpindleFunctionsDynamic>
                <Constant>
                    <value>{gamma['dyn']}</value>
                </Constant>
            </SpindleFunctionsDynamic>
        </SpindleController>
        """'''

new_xml = '''        spindle_ctrl = f"""
        <ControllerSet name="controllerset">
            <objects>
                <SpindleController name="spindle_controller">
                    <socket_actuators>muscle</socket_actuators>
                    <ControlFunctions>
                        <FunctionSet>
                            <objects>
                                <Constant>
                                    <value>0.1</value>
                                </Constant>
                            </objects>
                        </FunctionSet>
                    </ControlFunctions>
                    <SpindleFunctionsStatic>
                        <FunctionSet>
                            <objects>
                                <Constant>
                                    <value>{gamma['stat']}</value>
                                </Constant>
                            </objects>
                        </FunctionSet>
                    </SpindleFunctionsStatic>
                    <SpindleFunctionsDynamic>
                        <FunctionSet>
                            <objects>
                                <Constant>
                                    <value>{gamma['dyn']}</value>
                                </Constant>
                            </objects>
                        </FunctionSet>
                    </SpindleFunctionsDynamic>
                </SpindleController>
            </objects>
        </ControllerSet>
        """'''

if old_xml in content:
    content = content.replace(old_xml, new_xml)
    with open('test_scripts/fig3_ramp_stretches.py', 'w') as f:
        f.write(content)
    print('SUCCESS')
else:
    print('FAILED TO MATCH')

