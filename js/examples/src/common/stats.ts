import type { Panel } from 'dashcat';
import { getNumActiveBodies, type World } from 'tumble.js';
import type { StepTime } from './loop';

/** the step's time and the bodies awake, in a folder of the panel. loop fills in the step's time */
export function addWorldStats(panel: Panel, world: World): StepTime {
    const stepTime: StepTime = { milliseconds: 0 };
    const folder = panel.folder('stats');
    folder.graph(() => stepTime.milliseconds, { label: 'step', unit: 'duration', min: 0 });
    folder.monitor(() => getNumActiveBodies(world), { label: 'awake bodies' });
    return stepTime;
}
