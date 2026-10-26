import { bench, group } from '@pmndrs/labs';
import { tenThousandBoxes } from '../scenarios/ten-thousand-boxes';
import { runWindow, warm } from '../scenarios/scenario';

group('ten-thousand-boxes @physics', () => {
    bench('ten-thousand-boxes', function* () {
        // build and warm-up are untimed. see scenarios/scenario.ts for why the window resets
        const instance = tenThousandBoxes.create();
        warm(tenThousandBoxes, instance);

        yield () => runWindow(tenThousandBoxes, instance);
    });
});
