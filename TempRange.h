#ifndef TEMPRANGE_H
#define TEMPRANGE_H


/**
 * @struct TempRange
 * @brief Defines the four points required to define a temperature into the categories:
 * OK, WARNING, CRITICAL & FAULT
 *                sp [0]         sp [1]            sp [2]                  sp [3]
 * ^^^^^ FAULT ^^^^^|      OK      |     WARNING     |       CRITIACAL       |^^^^ FAULT ^^^^
*/
struct TempRange {

    float* setPoints;

    TempRange ()
    {
        setPoints = new float [4];
    }

    TempRange ( const float& minOk, const float& maxOk, const float& minCrit, const float& maxCrit )
    {
        setPoints = new float [4];
        setPoints [0] = minOk;
        setPoints [1] = maxOk;
        setPoints [2] = minCrit;
        setPoints [3] = maxCrit;
    }

    ~TempRange ()
    {
        free ( setPoints );
    }
};

enum TempRangeStates {
    OKAY,
    WARN,
    CRIT,
    FAULT
};

/**
 * @fn getTempState
 * @brief check which region of a temperature range a given temperature is and return the category of the range it is within
 * 
 * @arg {float} temp      - temperature to check against range
 * @arg {TempRange} range - temperature range used to classify temp 
*/
TempRangeStates getTempState ( const float& temp, const TempRange& range )
{
    TempRangeStates state;

    if ( temp < range.setPoints [0] || temp > range.setPoints [3] )
    {
        state = FAULT; // outside of (0 - 3)
    } 
    else if ( temp <= range.setPoints [1] ) 
    {
        state = OKAY; // within [0 - 1]
    } 
    else if ( temp >= range.setPoints [2] ) 
    {
        state = CRIT; // within [2 - 3]
    } 
    else 
    {
        state = WARN; // within (1 - 2)
    }

    return state;
} 

#endif