/**
 * @license
 *
 * Copyright 2019 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Example of including Blockly with using Webpack with
 *               defaults: (English lang & JavaScript generator).
 * @author samelh@google.com (Sam El-Husseini)
 */

import * as Blockly from 'blockly';

document.addEventListener("DOMContentLoaded", function () {
    const workspace = Blockly.inject('blocklyDiv',
        {
            toolbox: document.getElementById('toolbox'),
            media: 'media/'
        });

    const lang = 'JavaScript';
    const button = document.getElementById('blocklyButton');
    button.addEventListener('click', function () {
        console.log(Blockly.Xml.workspaceToDom(workspace));
    })
});


Blockly.Blocks['root'] = {
    init: function () {
        this.appendDummyInput()
            .appendField('root')
            .appendField('Mode')
            .appendField(new Blockly.FieldDropdown([["one time", "one"], ["Specific number of times", "specific"], ["Infinite repetition", "infinite"]]))
            .appendField('Number of repetition')
            .appendField(new Blockly.FieldNumber())
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#FCA918")
    }
};

Blockly.Blocks['sequence'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('sequence')
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#FCA918")
    }
};

Blockly.Blocks['selector'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('selector')
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#FCA918")
    }
};

Blockly.Blocks['parallel'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('parallel')
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#FCA918")
    }
};

Blockly.Blocks['service'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('service')
            .appendField('name')
            .appendField(new Blockly.FieldTextInput())
            .appendField('parameters')
            .appendField(new Blockly.FieldTextInput())
        this.setColour("#5CB1D6")
    }
};

Blockly.Blocks['sleep'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('sleep')
            .appendField('duration')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#5CB1D6")
    }
};

Blockly.Blocks['condition'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('condition')
            .appendField('Status')
            .appendField(new Blockly.FieldDropdown([["Success", "Success"], ["Failure", "Failure"], ["Running", "Running"]]))
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#59C059")
    }
};

Blockly.Blocks['inverter'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('inverter')
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#59C059")
    }
};

Blockly.Blocks['timeout'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('timeout')
            .appendField('duration')
            .appendField(new Blockly.FieldNumber())
        this.appendStatementInput('DO')
            .appendField('');
        this.setColour("#59C059")
    }
};

Blockly.Blocks['end effector'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('end effector')
            .appendField('parameters')
            .appendField(new Blockly.FieldTextInput())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['execute'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('execute')
            .appendField('id')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['cartesian'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('cartesian')
            .appendField('id')
            .appendField(new Blockly.FieldNumber())
            .appendField('reliability')
            .appendField(new Blockly.FieldNumber())
            .appendField('eef')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['plan'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('plan')
            .appendField('id')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['record'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('record')
            .appendField('Identifier')
            .appendField(new Blockly.FieldTextInput())
            .appendField('Time')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#5CB1D6")
    }
};

Blockly.Blocks['replay'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('replay')
            .appendField('Identifier')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['navigate'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('navigate')
            .appendField('Identifier')
            .appendField(new Blockly.FieldNumber())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['coverage'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('coverage')
            .appendField('Robot width')
            .appendField(new Blockly.FieldNumber())
            .appendField('Points making up the polygon (optional)')
            .appendField(new Blockly.FieldTextInput())
        this.setColour("#4C97FF")
    }
};

Blockly.Blocks['patrol'] = {
    init: function () {
        this.setPreviousStatement(true)
        this.setNextStatement(true)
        this.appendDummyInput()
            .appendField('patrol')
            .appendField('Points making up the polygon (optional)')
            .appendField(new Blockly.FieldTextInput())
        this.setColour("#4C97FF")
    }
};