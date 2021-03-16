import { Vec3 } from './Vec3'
import { Quaternion } from './lnQuaternion'

describe('lnQuaternion', () => {

  test('conjugate', () => {
    const q = new Quaternion().set( 1,2,3,4)
    q.conjugate(q)
    expect(q.x).toBe(-1)
    expect(q.y).toBe(-2)
    expect(q.z).toBe(-3)
    expect(q.w).toBe(4)
  })

  test('inverse', () => {
    const q = new Quaternion(1, 2, 3, 4)
    const denominator = 1 * 1 + 2 * 2 + 3 * 3 + 4 * 4
    q.inverse(q)

    // Quaternion inverse is conjugate(q) / ||q||^2
    expect(q.x).toBe(-1 / denominator)
    expect(q.y).toBe(-2 / denominator)
    expect(q.z).toBe(-3 / denominator)
    expect(q.w).toBe(4 / denominator)
  })

// need tests for integrate

/*
  test('slerp', () => {
    var qa = new Quaternion()
    var qb = new Quaternion()
    qa.slerp(qb, 0.5, qb)
    expect(qa).toStrictEqual(qb)

    qa.setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4)
    qb.setFromAxisAngle(new Vec3(0, 0, 1), -Math.PI / 4)
    qa.slerp(qb, 0.5, qb)
    expect(qb).toStrictEqual(new Quaternion())
  })
*/
  test('set', () => {
    const q = new Quaternion().set(1, 2, 3, 4)
    q.set(5, 6, 7, 8)
    expect(q.x).toBe(5)
    expect(q.y).toBe(6)
    expect(q.z).toBe(7)
    expect(q.w).toBe(8)
  })

  test('toString', () => {
    const q = new Quaternion().set(1, 2, 3, 4)
    expect(q.toString()).toBe('1,2,3,4')
  })

  test('toArray', () => {
    const q = new Quaternion().set(1,2,3)
    const qa = q.toArray()
    expect(qa[0]).toBe(1)
    expect(qa[1]).toBe(2)
    expect(qa[2]).toBe(3)
    expect(qa.length).toBe(3)
  })

  test('copy', () => {
    const q = new Quaternion(1, 2, 3, 4)
    const qc = new Quaternion()
    qc.copy(q)
    q.set(4, 5, 6, 7)
    expect(qc.x).toBe(1)
    expect(qc.y).toBe(2)
    expect(qc.z).toBe(3)
    expect(qc.w).toBe(4)
  })

  test('clone', () => {
    const q = new Quaternion(1, 2, 3, 4)
    const qc = q.clone()
    q.set(4, 5, 6, 7)
    expect(qc.x).toBe(1)
    expect(qc.y).toBe(2)
    expect(qc.z).toBe(3)
    expect(qc.w).toBe(4)
  })
})
