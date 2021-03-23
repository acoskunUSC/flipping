package tt.euclidtime3i.sipp.intervals;

import org.junit.Before;
import org.junit.Test;

import tt.euclidtime3i.sipp.intervals.Interval;
import tt.euclidtime3i.sipp.intervals.SafeIntervalList;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import static org.junit.Assert.assertTrue;

public class SafeIntervalBuilderTest {

    private SafeIntervalList intervals;
    private List<Interval> expected;

    @Before
    public void setUp() throws Exception {
        intervals = new SafeIntervalList(Integer.MAX_VALUE);
        expected = new LinkedList<Interval>();
    }

    @Test
    public void testWithZero() throws Exception {
        intervals.markCollisionInTime(0, 0);
        intervals.markCollisionInTime(1, 1);
        intervals.markCollisionInTime(2, 2);
        intervals.markCollisionInTime(5, 5);
        intervals.markCollisionInTime(6, 6);
        intervals.markCollisionInTime(9, 9);

        expected.add(new Interval(2, 5));
        expected.add(new Interval(6, 9));
        expected.add(new Interval(9, Integer.MAX_VALUE));

        checkEquals(intervals.iterator(), expected.iterator());
    }

    @Test
    public void testWithoutZero() throws Exception {
        intervals.markCollisionInTime(1, 1);
        intervals.markCollisionInTime(2, 2);
        intervals.markCollisionInTime(5, 5);
        intervals.markCollisionInTime(6, 6);
        intervals.markCollisionInTime(9, 9);

        expected.add(new Interval(0, 1));
        expected.add(new Interval(2, 5));
        expected.add(new Interval(6, 9));
        expected.add(new Interval(9, Integer.MAX_VALUE));

        checkEquals(intervals.iterator(), expected.iterator());
    }

    private void checkEquals(Iterator<Interval> itA, Iterator<Interval> itB) {
        while (itA.hasNext() && itB.hasNext()) {
            Interval a = itA.next();
            Interval b = itB.next();

            //System.out.printf("%s %s%n", a, b);
            assertTrue(a.equals(b));
        }

        assertTrue(itA.hasNext() == itB.hasNext());
    }
}
